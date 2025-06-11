#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import threading
import time

class IndividualErrorPlotter:
    def __init__(self):
        rospy.init_node('individual_error_plotter', anonymous=True)
        
        # Diccionarios para almacenar datos por drone_id
        self.drone_data = defaultdict(lambda: {
            'times': [],
            'consensus_errors': [],
            'trajectory_errors': []
        })
        
        # Marca de tiempo inicial (offset)
        self.start_time = None
        self.num_drones = 5  # Leader (0) + 4 followers
        self.colors = ['red', 'blue', 'green', 'orange', 'purple']
        self.drone_names = ['Líder', 'Seguidor 1', 'Seguidor 2', 'Seguidor 3', 'Seguidor 4']
        
        # Variables de control de threading y actualización
        self.data_lock = threading.Lock()
        self.new_data_available = False
        self.last_update_time = time.time()
        
        # Configuración de las figuras (no en modo interactivo aún)
        self.setup_plots()
        
        # Suscripción a métricas individuales
        rospy.Subscriber('/individual_metrics', Float64MultiArray, self.individual_metrics_callback)
        
        rospy.loginfo("Individual Error Plotter inicializado")
    
    def setup_plots(self):
        # Configuración de subplots
        self.fig, (self.consensus_ax, self.trajectory_ax) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Gráfico de errores de consenso
        self.consensus_ax.set_xlabel('Tiempo [s]')
        self.consensus_ax.set_ylabel('Error de Consenso [m]')
        self.consensus_ax.set_title('Errores de Consenso Individuales por Dron')
        self.consensus_ax.grid(True, alpha=0.3)
        self.consensus_ax.set_ylim(0, 2.0)
        
        # Gráfico de errores de trayectoria
        self.trajectory_ax.set_xlabel('Tiempo [s]')
        self.trajectory_ax.set_ylabel('Error de Trayectoria [m]')
        self.trajectory_ax.set_title('Errores de Trayectoria Individuales por Dron')
        self.trajectory_ax.grid(True, alpha=0.3)
        self.trajectory_ax.set_ylim(0, 3.0)
        
        # Inicializar líneas para cada dron
        self.consensus_lines = {}
        self.trajectory_lines = {}
        for i in range(self.num_drones):
            line_cons, = self.consensus_ax.plot([], [], color=self.colors[i], linewidth=2,
                                                label=self.drone_names[i], marker='o', markersize=3, alpha=0.8)
            self.consensus_lines[i] = line_cons
            line_traj, = self.trajectory_ax.plot([], [], color=self.colors[i], linewidth=2,
                                                 label=self.drone_names[i], marker='s', markersize=3, alpha=0.8)
            self.trajectory_lines[i] = line_traj
        
        # Leyendas
        self.consensus_ax.legend(loc='upper right', fontsize=10)
        self.trajectory_ax.legend(loc='upper right', fontsize=10)
        plt.tight_layout()
    
    def individual_metrics_callback(self, msg):
        if len(msg.data) != 4:
            rospy.logwarn_throttle(5, "Mensaje individual_metrics con formato incorrecto: %d elementos (esperado 4)", len(msg.data))
            return
        
        # Parsear mensaje: [drone_id, consensus_error, trajectory_error, timestamp]
        drone_id = int(msg.data[0])
        consensus_error = msg.data[1]
        trajectory_error = msg.data[2]
        timestamp = msg.data[3]
        
        # Validar drone_id
        if drone_id < 0 or drone_id >= self.num_drones:
            rospy.logwarn_throttle(5, "drone_id inválido: %d", drone_id)
            return
        
        # Establecer tiempo inicial (offset) en el primer mensaje
        if self.start_time is None:
            self.start_time = timestamp
            rospy.loginfo("Tiempo inicial establecido: %.2f", self.start_time)
        
        relative_time = timestamp - self.start_time
        
        with self.data_lock:
            # Almacenar datos sin eliminar históricos
            self.drone_data[drone_id]['times'].append(relative_time)
            self.drone_data[drone_id]['consensus_errors'].append(consensus_error)
            self.drone_data[drone_id]['trajectory_errors'].append(trajectory_error)
            
            # Marcar que hay nuevos datos
            self.new_data_available = True
    
    def update_plots(self):
        """Actualizar los gráficos con los datos más recientes"""
        with self.data_lock:
            # Límites dinámicos
            max_consensus = 0.1
            max_trajectory = 0.1
            min_time = float('inf')
            max_time = float('-inf')
            
            # Actualizar cada dron
            for drone_id in range(self.num_drones):
                data = self.drone_data.get(drone_id)
                if not data or not data['times']:
                    continue
                t = data['times']
                c = data['consensus_errors']
                tr = data['trajectory_errors']
                self.consensus_lines[drone_id].set_data(t, c)
                self.trajectory_lines[drone_id].set_data(t, tr)
                max_consensus = max(max_consensus, max(c))
                max_trajectory = max(max_trajectory, max(tr))
                min_time = min(min_time, t[0])
                max_time = max(max_time, t[-1])
            
            self.new_data_available = False
        
        # Ajuste de ejes
        if min_time < max_time:
            margin_t = max(1.0, 0.05*(max_time - min_time))
            self.consensus_ax.set_xlim(min_time - margin_t, max_time + margin_t)
            self.trajectory_ax.set_xlim(min_time - margin_t, max_time + margin_t)
            self.consensus_ax.set_ylim(0, max_consensus * 1.1)
            self.trajectory_ax.set_ylim(0, max_trajectory * 1.1)
        
        # Redibujar sin borrar datos previos
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def print_statistics(self):
        last_stats = time.time()
        while not rospy.is_shutdown():
            if time.time() - last_stats >= 10.0:
                rospy.loginfo("=== ESTADÍSTICAS DE ERROR ===")
                with self.data_lock:
                    for drone_id, data in self.drone_data.items():
                        if not data['consensus_errors']:
                            continue
                        cons = data['consensus_errors'][-50:]
                        traj = data['trajectory_errors'][-50:]
                        rospy.loginfo("%s - Consenso: Avg=%.3f, Max=%.3f | Trayectoria: Avg=%.3f, Max=%.3f",
                                      self.drone_names[drone_id], np.mean(cons), np.max(cons), np.mean(traj), np.max(traj))
                last_stats = time.time()
            time.sleep(0.1)
    
    def spin(self):
        # Thread de estadísticas
        stats_thread = threading.Thread(target=self.print_statistics)
        stats_thread.daemon = True
        stats_thread.start()
        
        # Mostrar gráficas sin bloquear
        plt.ion()
        plt.show(block=False)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.new_data_available and time.time() - self.last_update_time > 0.2:
                try:
                    self.update_plots()
                    self.last_update_time = time.time()
                except Exception as e:
                    rospy.logerr("Error actualizando gráficos: %s", str(e))
            plt.pause(0.01)
            rate.sleep()

if __name__ == '__main__':
    try:
        plotter = IndividualErrorPlotter()
        plotter.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.close('all')
