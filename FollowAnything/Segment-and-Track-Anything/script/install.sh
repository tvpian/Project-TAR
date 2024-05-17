# Install SAM
cd sam; pip3 install -e .
cd -

# Install other lib
pip3 install numpy opencv-python pycocotools matplotlib Pillow scikit-image

# Install Pytorch Correlation
git clone https://github.com/ClementPinard/Pytorch-Correlation-extension.git
cd Pytorch-Correlation-extension; python3 setup.py install
cd -
