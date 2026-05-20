#!/bin/bash
set -e

# Get the directory where THIS script is located
# This ensures it finds dt.py regardless of where the user downloaded the folder
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "--- Step 1: System Packages ---"
# Wait for system locks
while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 ; do
    echo "Waiting for other updates to finish..."
    sleep 2
done

sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y build-essential git cmake python3 python3-pip python3-dev \
                        libboost-all-dev libssl-dev libsqlite3-dev \
                        g++ clang ninja-build pkg-config

echo "--- Step 2: Python Dependencies ---"
pip3 install --upgrade pip
pip3 install cppyy
python3 -c "import cppyy; print('✅ cppyy pre-installed:', cppyy.__version__)"

echo "--- Step 3: Setup ns-3.46 ---"
# Install ns-3 in the user's home folder to ensure write permissions
NS3_INSTALL_DIR="$HOME/ns3_setup"
mkdir -p "$NS3_INSTALL_DIR"
cd "$NS3_INSTALL_DIR"

if [ ! -d "ns-3-dev" ]; then
   echo "Cloning ns-3 source..."
   git clone https://gitlab.com/nsnam/ns-3-dev.git
fi

cd ns-3-dev
git checkout ns-3.46 2>/dev/null || git checkout ns-3.41

echo "--- Step 4: Configure & Build ns-3 ---"
./ns3 configure --enable-python-bindings \
  --enable-modules=core,network,mobility,antenna,propagation,spectrum,lte \
  --build-profile=debug
./ns3 build -j$(nproc)

echo "--- Step 5: Configure Shared Library Paths ---"
echo "$NS3_INSTALL_DIR/ns-3-dev/build/lib" | sudo tee /etc/ld.so.conf.d/ns3.conf
sudo ldconfig

echo "--- Step 6: Launching Digital Twin ---"
# Go back to the folder where install.sh and dt.py are
cd "$SCRIPT_DIR"

if [ -f "dt.py" ]; then
    echo "🚀 Starting Digital Twin..."
    python3 dt.py
else
    echo "❌ Error: dt.py not found in $SCRIPT_DIR"
    exit 1
fi
