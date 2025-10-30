# work_3

### core build libraries
sudo apt update
sudo apt install -y build-essential cmake pkg-config git

### rtsp libraries
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstrtspserver-1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools

### yaml lib
sudo apt install -y libyaml-cpp-dev

### boost
sudo apt install -y libboost-all-dev

### openssl
sudo apt install -y libssl-dev

### JSON
sudo apt install -y nlohmann-json3-dev

### mavlink router command
./mavp2p udpc:192.168.144.13:4000 udps:127.0.0.1:14600


