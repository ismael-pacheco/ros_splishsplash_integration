#!/bin/bash
# install_ruby.sh - Instalación mínima de Ruby 1.9.3 con rbenv

set -e

echo "[1/3] Instalando dependencias básicas..."
sudo apt-get update
sudo apt-get install -y \
  git curl autoconf bison build-essential \
  libssl-dev libyaml-dev libreadline-dev \
  zlib1g-dev libncurses5-dev libffi-dev \
  libgdbm-dev libsqlite3-dev

echo "[2/3] Configurando rbenv..."

# Corregido: Usar $HOME en lugar de ~ en las condiciones
if [ ! -d "$HOME/.rbenv" ]; then
  git clone https://github.com/rbenv/rbenv.git ~/.rbenv
else
  cd ~/.rbenv
  git checkout master
  git pull
fi

cd ~/.rbenv && src/configure && make -C src
echo 'export PATH="$HOME/.rbenv/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(rbenv init -)"' >> ~/.bashrc
source ~/.bashrc

# Corregido: Usar $HOME en lugar de ~ en las condiciones
if [ ! -d "$HOME/.rbenv/plugins/ruby-build" ]; then
  git clone https://github.com/rbenv/ruby-build.git ~/.rbenv/plugins/ruby-build
else
  cd ~/.rbenv/plugins/ruby-build
  git checkout master
  git pull
fi
cd ~
echo "[3/3] Instalando Ruby 1.9.3-p551..."
RUBY_CONFIGURE_OPTS="--with-openssl-dir=/usr/lib/ssl" rbenv install 1.9.3-p551 || {
  echo "Error al instalar Ruby 1.9.3-p551"
  echo "Intentando con OpenSSL alternativo..."
  # Instalar OpenSSL 1.0.2u en un directorio local
  mkdir -p ~/openssl
  cd ~/openssl
if [ ! -f "openssl-1.0.2u.tar.gz" ]; then
  wget https://www.openssl.org/source/openssl-1.0.2u.tar.gz
  tar xzf openssl-1.0.2u.tar.gz
else
  tar xzf openssl-1.0.2u.tar.gz
fi  
  cd openssl-1.0.2u
  ./config --prefix=$HOME/.openssl --openssldir=$HOME/.openssl
  make && make install
  # Reintentar instalación de Ruby
  RUBY_CONFIGURE_OPTS="--with-openssl-dir=$HOME/.openssl" rbenv install 1.9.3-p551
}

rbenv global 1.9.3-p551

echo "¡Instalación completada!"
echo "Versión de Ruby: $(ruby -v)"
