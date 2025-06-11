#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Bool
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import threading

class SimplePlotter:
    def __init__(self):
        self.lock = threading.Lock()
        self.start_time = None
        self.adaptive_switch_time = None
        
        # Datos con timestamps relativos - sin límite máximo
        self.desired_pos = {'t': deque(), 'x': deque(), 'y': deque(), 'z': deque()}
        self.real_pos = {'t': deque(), 'x': deque(), 'y': deque(), 'z': deque()}
        self.adaptive_err = {'t': deque(), 'ex': deque(), 'ey': deque(), 'ez': deque()}
        self.pos_err = {'t': deque(), 'ex': deque(), 'ey': deque(), 'ez': deque()}
        self.inertia_diff = {'t': deque(), 'dxx': deque(), 'dyy': deque(), 'dzz': deque()}
        
        self.adaptive_mode = False
        self.done_received = False
        self.plot_ready = False
        self.lines = {}  # Almacenar referencias a las líneas para actualización eficiente
        self.traj_3d_initialized = False  # Flag para inicializar scatter plot una sola vez
        
        plt.ion()
        self.setup_subscribers()
        self.setup_plots()
        
        rospy.loginfo("Graficador simplificado iniciado")

    def setup_subscribers(self):
        rospy.Subscriber("/desired/pose", PoseStamped, self.cb_desired_pose, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_real_pose, queue_size=10)
        rospy.Subscriber("/adaptive/error", Vector3Stamped, self.cb_adaptive_error, queue_size=50)
        rospy.Subscriber("/trajectory/position_error", Vector3Stamped, self.cb_position_error, queue_size=10)
        rospy.Subscriber("/adaptive/inertia_diff", Vector3Stamped, self.cb_inertia_diff, queue_size=50)
        rospy.Subscriber("/adaptive/done", Bool, self.cb_done, queue_size=1)
        rospy.Subscriber("/adaptive/mode_switch", Bool, self.cb_adaptive_switch, queue_size=1)

    def setup_plots(self):
        self.fig, self.axes = plt.subplots(2, 3, figsize=(16, 10))
        self.fig.suptitle('Control Adaptativo - Monitoreo en Tiempo Real', fontsize=14)
        
        # Configurar títulos y etiquetas
        titles = [['Trayectoria XY', 'Altitud vs Tiempo', 'Errores de Posición'],
                 ['Errores Adaptativos', 'Diferencias de Inercia', 'Trayectoria 3D']]
        
        for i in range(2):
            for j in range(3):
                self.axes[i,j].set_title(titles[i][j])
                self.axes[i,j].grid(True, alpha=0.3)
        
        # Configurar ejes específicos
        self.axes[0,0].set_xlabel('X [m]'); self.axes[0,0].set_ylabel('Y [m]')
        self.axes[0,1].set_xlabel('Tiempo [s]'); self.axes[0,1].set_ylabel('Z [m]')
        self.axes[0,2].set_xlabel('Tiempo [s]'); self.axes[0,2].set_ylabel('Error [m]')
        self.axes[1,0].set_xlabel('Tiempo [s]'); self.axes[1,0].set_ylabel('Error [rad/s]')
        self.axes[1,1].set_xlabel('Tiempo [s]'); self.axes[1,1].set_ylabel('ΔInercia [kg⋅m²]')
        self.axes[1,2].set_xlabel('X [m]'); self.axes[1,2].set_ylabel('Y [m]')
        
        # Configurar límites fijos para la trayectoria 3D
        self.axes[1,2].set_xlim([-6.5, 6.5])
        self.axes[1,2].set_ylim([-6.5, 6.5])
        
        # Inicializar líneas vacías para actualización eficiente
        self.init_empty_lines()
        
        plt.tight_layout()
        # NO mostrar aquí - se mostrará en run() cuando haya datos
        self.plot_ready = True
        rospy.loginfo("Gráficas configuradas, esperando datos para mostrar ventana")

    def init_empty_lines(self):
        """Crear líneas vacías para actualización eficiente sin borrar datos"""
        # Trayectoria XY
        self.lines['des_xy'], = self.axes[0,0].plot([], [], 'b-', linewidth=2, label='Deseada')
        self.lines['real_xy'], = self.axes[0,0].plot([], [], 'r-', linewidth=2, label='Real')
        self.lines['current_pos'], = self.axes[0,0].plot([], [], 'ro', markersize=8, label='Actual')
        
        # Altitud
        self.lines['des_z'], = self.axes[0,1].plot([], [], 'b-', linewidth=2, label='Z Deseada')
        self.lines['real_z'], = self.axes[0,1].plot([], [], 'r-', linewidth=2, label='Z Real')
        
        # Errores de posición
        self.lines['pos_err_x'], = self.axes[0,2].plot([], [], 'r-', label='Error X')
        self.lines['pos_err_y'], = self.axes[0,2].plot([], [], 'g-', label='Error Y')
        self.lines['pos_err_z'], = self.axes[0,2].plot([], [], 'b-', label='Error Z')
        
        # Errores adaptativos
        self.lines['adapt_err_x'], = self.axes[1,0].plot([], [], 'r-', linewidth=2, label='Roll')
        self.lines['adapt_err_y'], = self.axes[1,0].plot([], [], 'g-', linewidth=2, label='Pitch')
        self.lines['adapt_err_z'], = self.axes[1,0].plot([], [], 'b-', linewidth=2, label='Yaw')
        
        # Diferencias de inercia
        self.lines['inertia_xx'], = self.axes[1,1].plot([], [], 'r-', linewidth=2, label='ΔIxx')
        self.lines['inertia_yy'], = self.axes[1,1].plot([], [], 'g-', linewidth=2, label='ΔIyy')
        self.lines['inertia_zz'], = self.axes[1,1].plot([], [], 'b-', linewidth=2, label='ΔIzz')
        self.lines['inertia_ref'], = self.axes[1,1].plot([], [], 'k--', alpha=0.5, label='Ref')
        
        # Inicializar trayectoria 3D sin datos
        self.lines['traj_3d_scatter'] = None
        self.lines['traj_3d_start'] = None
        self.lines['traj_3d_current'] = None
        
        # Agregar leyendas
        for ax in self.axes.flat:
            ax.legend(loc='upper right', fontsize=8)

    def get_relative_time(self, stamp):
        if self.start_time is None:
            self.start_time = stamp
            return 0.0
        return (stamp - self.start_time).to_sec()

    def cb_desired_pose(self, msg):
        with self.lock:
            t = self.get_relative_time(msg.header.stamp)
            self.desired_pos['t'].append(t)
            self.desired_pos['x'].append(msg.pose.position.x)
            self.desired_pos['y'].append(msg.pose.position.y)
            self.desired_pos['z'].append(msg.pose.position.z)

    def cb_real_pose(self, msg):
        with self.lock:
            t = self.get_relative_time(msg.header.stamp)
            self.real_pos['t'].append(t)
            self.real_pos['x'].append(msg.pose.position.x)
            self.real_pos['y'].append(msg.pose.position.y)
            self.real_pos['z'].append(msg.pose.position.z)

    def cb_adaptive_error(self, msg):
        with self.lock:
            t = self.get_relative_time(msg.header.stamp)
            self.adaptive_err['t'].append(t)
            self.adaptive_err['ex'].append(msg.vector.x)
            self.adaptive_err['ey'].append(msg.vector.y)
            self.adaptive_err['ez'].append(msg.vector.z)

    def cb_position_error(self, msg):
        with self.lock:
            t = self.get_relative_time(msg.header.stamp)
            self.pos_err['t'].append(t)
            self.pos_err['ex'].append(msg.vector.x)
            self.pos_err['ey'].append(msg.vector.y)
            self.pos_err['ez'].append(msg.vector.z)

    def cb_inertia_diff(self, msg):
        with self.lock:
            t = self.get_relative_time(msg.header.stamp)
            self.inertia_diff['t'].append(t)
            self.inertia_diff['dxx'].append(msg.vector.x)
            self.inertia_diff['dyy'].append(msg.vector.y)
            self.inertia_diff['dzz'].append(msg.vector.z)

    def cb_adaptive_switch(self, msg):
        if msg.data and not self.adaptive_mode:
            self.adaptive_mode = True
            if self.start_time:
                self.adaptive_switch_time = self.get_relative_time(rospy.Time.now())
                rospy.loginfo(f"Modo ADAPTATIVO activado en t={self.adaptive_switch_time:.2f}s")

    def cb_done(self, msg):
        if msg.data:
            self.done_received = True
            rospy.loginfo("Trayectoria completada")

    def update_plots(self):
        """Actualizar gráficas SIN BORRAR datos antiguos - PRESERVAR HISTORIAL COMPLETO"""
        if not self.plot_ready or not self.start_time:
            return
        
        try:
            with self.lock:
                # 1. Actualizar trayectoria XY
                if len(self.desired_pos['x']) > 0:
                    self.lines['des_xy'].set_data(list(self.desired_pos['x']), list(self.desired_pos['y']))
                    
                if len(self.real_pos['x']) > 0:
                    self.lines['real_xy'].set_data(list(self.real_pos['x']), list(self.real_pos['y']))
                    # Posición actual
                    self.lines['current_pos'].set_data([self.real_pos['x'][-1]], [self.real_pos['y'][-1]])
                    
                # Auto-escalar XY
                self.axes[0,0].relim()
                self.axes[0,0].autoscale()

                # 2. Actualizar altitud - PRESERVAR TODOS LOS DATOS
                if len(self.desired_pos['t']) > 0:
                    self.lines['des_z'].set_data(list(self.desired_pos['t']), list(self.desired_pos['z']))
                if len(self.real_pos['t']) > 0:
                    self.lines['real_z'].set_data(list(self.real_pos['t']), list(self.real_pos['z']))
                    
                self.axes[0,1].relim()
                self.axes[0,1].autoscale()

                # 3. Errores de posición - PRESERVAR TODOS LOS DATOS
                if len(self.pos_err['t']) > 0:
                    self.lines['pos_err_x'].set_data(list(self.pos_err['t']), list(self.pos_err['ex']))
                    self.lines['pos_err_y'].set_data(list(self.pos_err['t']), list(self.pos_err['ey']))
                    self.lines['pos_err_z'].set_data(list(self.pos_err['t']), list(self.pos_err['ez']))
                    
                self.axes[0,2].relim()
                self.axes[0,2].autoscale()

                # 4. ERRORES ADAPTATIVOS - CRÍTICO: NUNCA BORRAR ESTOS DATOS
                if len(self.adaptive_err['t']) > 0:
                    # Actualizar líneas con TODOS los datos históricos
                    t_adapt = list(self.adaptive_err['t'])
                    ex_adapt = list(self.adaptive_err['ex'])
                    ey_adapt = list(self.adaptive_err['ey'])
                    ez_adapt = list(self.adaptive_err['ez'])
                    
                    self.lines['adapt_err_x'].set_data(t_adapt, ex_adapt)
                    self.lines['adapt_err_y'].set_data(t_adapt, ey_adapt)
                    self.lines['adapt_err_z'].set_data(t_adapt, ez_adapt)
                    
                    # Log para verificar que no se pierden datos
                    if len(t_adapt) % 50 == 0:  # Log cada 50 puntos
                        rospy.loginfo_throttle(2, f"Errores adaptativos: {len(t_adapt)} puntos almacenados")
                    
                self.axes[1,0].relim()
                self.axes[1,0].autoscale()

                # 5. Diferencias de inercia - PRESERVAR TODOS LOS DATOS
                if len(self.inertia_diff['t']) > 0:
                    t_list = list(self.inertia_diff['t'])
                    dxx_list = list(self.inertia_diff['dxx'])
                    dyy_list = list(self.inertia_diff['dyy'])
                    dzz_list = list(self.inertia_diff['dzz'])
                    
                    self.lines['inertia_xx'].set_data(t_list, dxx_list)
                    self.lines['inertia_yy'].set_data(t_list, dyy_list)
                    self.lines['inertia_zz'].set_data(t_list, dzz_list)
                    
                    # Línea de referencia
                    if len(t_list) > 1:
                        self.lines['inertia_ref'].set_data([t_list[0], t_list[-1]], [0, 0])
                        
                self.axes[1,1].relim()
                self.axes[1,1].autoscale()

                # 6. Trayectoria 3D - ACTUALIZAR SIN BORRAR (usando offsets en lugar de clear)
                if len(self.real_pos['x']) > 5:
                    x_vals = np.array(list(self.real_pos['x']))
                    y_vals = np.array(list(self.real_pos['y']))
                    z_vals = np.array(list(self.real_pos['z']))
                    
                    # Si es la primera vez, crear los elementos
                    if not self.traj_3d_initialized:
                        self.lines['traj_3d_scatter'] = self.axes[1,2].scatter(
                            x_vals, y_vals, c=z_vals, cmap='viridis', 
                            s=10, alpha=0.7, label='Trayectoria')
                        self.lines['traj_3d_start'], = self.axes[1,2].plot(
                            x_vals[0], y_vals[0], 'go', markersize=8, label='Inicio')
                        self.lines['traj_3d_current'], = self.axes[1,2].plot(
                            x_vals[-1], y_vals[-1], 'ro', markersize=8, label='Actual')
                        self.axes[1,2].legend()
                        self.traj_3d_initialized = True
                    else:
                        # Actualizar datos existentes sin borrar
                        self.lines['traj_3d_scatter'].set_offsets(np.column_stack((x_vals, y_vals)))
                        self.lines['traj_3d_scatter'].set_array(z_vals)
                        self.lines['traj_3d_current'].set_data([x_vals[-1]], [y_vals[-1]])
                    
                    # No autoescalar - mantener límites fijos
                    # self.axes[1,2].relim()
                    # self.axes[1,2].autoscale()

                # Agregar marcadores de cambio de modo si existe (una sola vez)
                if self.adaptive_switch_time:
                    for ax in [self.axes[0,1], self.axes[0,2], self.axes[1,0], self.axes[1,1]]:
                        if not hasattr(ax, '_switch_line_added'):
                            ax.axvline(x=self.adaptive_switch_time, color='red', linestyle='--', 
                                     linewidth=2, alpha=0.8, label='Modo Adaptativo')
                            ax._switch_line_added = True
                            ax.legend(loc='upper right', fontsize=8)  # Refrescar leyenda

                plt.draw()
                plt.pause(0.001)  # Forzar actualización de la ventana
                
        except Exception as e:
            rospy.logwarn_throttle(5, f"Error actualizando gráficas: {e}")

    def run(self):
        rospy.loginfo("Esperando datos para mostrar ventana...")
        
        # Esperar a recibir primeros datos
        while not rospy.is_shutdown() and self.start_time is None:
            rospy.sleep(0.1)
        
        if rospy.is_shutdown():
            return
            
        rospy.loginfo("Datos recibidos - Mostrando ventana de gráficas")
        rospy.loginfo("MODO DE PRESERVACIÓN DE DATOS ACTIVADO - No se borrarán datos históricos")
        
        # Mostrar ventana y mantenerla activa
        plt.show(block=False)
        plt.pause(0.1)  # Asegurar que la ventana aparezca
        
        # Timer de actualización
        rate = rospy.Rate(5)  # 5 Hz para evitar sobrecarga
        
        while not rospy.is_shutdown():
            try:
                # Verificar si la ventana sigue abierta
                if not plt.fignum_exists(self.fig.number):
                    rospy.logwarn("Ventana cerrada por el usuario")
                    break
                    
                self.update_plots()
                plt.pause(0.001)  # Permitir eventos de matplotlib
                rate.sleep()
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Error en loop principal: {e}")
                break
        
        plt.close('all')
        rospy.loginfo("Graficador cerrado correctamente")


if __name__ == "__main__":
    try:
        rospy.init_node("simple_adaptive_plotter", anonymous=True)
        plotter = SimplePlotter()
        plotter.run()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")