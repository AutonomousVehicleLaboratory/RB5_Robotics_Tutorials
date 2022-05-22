---
title: (1) Running DeepLabV3 Model
categories:
  - 4 ML at the Edge
tags:
  - Deep Learning
date: 2022-05-18 17:12:21
---

This tutorial explains the process of setting up the SNPE SDK and running inference on RB5 using a TensorFlow and PyTorch segmentation model. 

Note: This can be extended to any Deep Learning models

## TesnorFlow

### Running Inference on Ubuntu 18.0.4:

This section will guide you in setting up the SNPE on a Ubuntu system and running inference using the TensorFlow model for DeepLabV3

1. Download pre-trained DeepLabV3 model trained using TensorFlow:

```
wget http://download.tensorflow.org/modelsdeeplabv3_mnv2_pascal_train_aug_2018_01_29.tar.gz
tar -xzvf deeplabv3_mnv2_pascal_train_aug_2018_01_29.tar.gz
```

2. Setup Qualcomm Snapdragon Neural Processing Engine SDK on the system using the tutorial mentioned below.

```
sudo snap install --classic android-studio #Android Studio installation is necessary for SNPE SDK to work
https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/getting-started
```
Note: Make sure all the path variables are set properly according to the tutorial provided in the above links. Failing to set the paths will result in the following command to fail.

3. Set the environment path for TensorFlow

```
cd $SNPE_ROOT
export TENSORFLOW_DIR="your_tensorflow_installation_dir"
source bin/envsetup.sh -o $TENSORFLOW_DIR
```


4. Convert the model to .dlc format using the following command 

```
snpe-tensorflow-to-dlc --input_dim sub_7 1,513,513,3 --out_node ArgMax --input_network ./deeplabv3_mnv2_pascal_train_aug/frozen_inference_graph.pb
```
Note: The "./deeplabv3_mnv2_pascal_train_aug/frozen_inference_graph.pb" is the downloaded TF model and the image size is set to 513x513x3 as an example

5. Running Inference:
- Preprocess the image using the Python script below. Example image is provided.
```
import numpy as np
import cv2
from matplotlib import pyplot as plt

frame = cv2.imread('Example.jpeg')
# Resize frame with Required image size
frame_resized = cv2.resize(frame,(513,513))
# Pad smaller dimensions to Mean value & Multiply with 0.007843
blob = cv2.dnn.blobFromImage(frame_resized, 0.007843, (513, 513), (127.5, 127.5, 127.5), swapRB=True)

# Making numpy array of required shape
blob = np.reshape(blob, (1,513,513,3))

# Storing to a raw file
np.ndarray.tofile(blob, open('blob.raw','w') )
```

- Prepare a text file that contains all the images you would like to run inference on
    - Create a file names "raw_list.txt" in the current directory
    - Enter the path of the "blob.raw" file that was generated using the Python script

- Run the following command to use the generated dlc model to run inference

```
snpe-net-run --container deeplabv3.dlc --input_list ./raw_list.txt
```
This command generates a file called "ArgMax:0.raw" in “/output/Result_0/” path that will be used as an input to out model.

- Run the input below Python script to obtain the segmentation masks and modify the image

```
import cv2
import numpy as np
from matplotlib import pyplot as plt

arr = np.fromfile(open('ArgMax:0.raw', 'r'), dtype="float32")
arr = np.reshape(arr, (513,513,1))
segment = arr[342:, 342:]
arr[arr == 15] = 255
original_img = cv2.imread('deeplab-check.jpeg')
arr2=cv2.resize(segment,(original_img.shape[1], original_img.shape[0]))
print(arr.shape)
for i in range(arr2.shape[0]):
    for j in range(arr2.shape[1]):
        if (arr2[i][j] != 255):
            original_img[i][j] = original_img[i][j][0] = original_img[i][j][1] = original_img[i][j][2]
plt.imshow(original_img)
plt.show()
plt.imshow( arr, cmap="gray")
plt.show()
```

### Running Inference on RB5:

The SNPE SDK provides binaries for RB5's architecture. To check out the list of supported architectures, run

#### On Ubuntu:
```
cd $SNPE_ROOT/lib #Ensure the path export from SNPE installation
ls
```

For the Qualcomm RB5 platform, we are interested in in the following folders:
- aarch64-ubuntu-gcc7.5
- dsp

These folders need to be copied over to the RB5 either by using "adb shell", "adb push" or "scp" commands.

1. Select the architecture aarch64-ubuntu-gcc7.5

```
export SNPE_TARGET_ARCH=aarch64-ubuntu-gcc7.5
```

2. Push the binaries to target

```
adb shell "mkdir -p /data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/bin" # Creates a folder with the architecture's name
adb shell "mkdir -p /data/local/tmp/snpeexample/dsp/lib" #Creates lib folder to copy over the libraries to
adb push $SNPE_ROOT/lib/$SNPE_TARGET_ARCH/*.so /data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/lib #Copy the architecture libraries
adb push $SNPE_ROOT/lib/dsp/*.so  /data/local/tmp/snpeexample/dsp/lib
adb push $SNPE_ROOT/bin/$SNPE_TARGET_ARCH/snpe-net-run /data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/bin
```

Once the libraries are copied over, log into RB5 using "adb shell" command or use a monitor(preferred as the final result involves visualizing)

#### On RB5:
1. Set tup the target architecture, library path and environment variables for "snpe-net-run" command to run successfully

```
export SNPE_TARGET_ARCH=aarch64-ubuntu-gcc7.5
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/lib
export PATH=$PATH:/data/local/tmp/snpeexample/$SNPE_TARGET_ARCH/bin
```

Note: These commands need to be run everytime a new terminal is opened. To avoid this, add these commands in ~/.bashrc file and run "source ~/.bashrc"

2. Verify snpe-net-run copy

```
snpe-net-run -h #This command should run successfully and list the available options
```

3. Copy the Python scripts and the ".dlc" file and the images from "Running Inference on Ubuntu 18.0.4" section to run inference

4. Follow Step 4 from the previous section to run inference. (Feel free to skip the blob.raw generation if it is already copied over)

Note: The inference step involves running "snpe-net-run".

Note: Once the masks are obtained, it can be used for any application. We have shown a simple background blur in this example.

## PyTorch

A PyTorch model can be converted to dlc format to be run on RB5 as mentioned in the following sections. 

Important: PyTorch models need to be converted to ONNX before they are converted to dlc format.

1. Run the following script to generate DeepLabV3 ONNX model. Here we use pre-trained DeepLabV3 model available in TorchHub

```
import torch
import torchvision
BEST_MODEL_PATH_ONNX = "deeplabv3_onnx_model.onnx"
#Load pre-trained Model
model = torch.hub.load('pytorch/vision:v0.7.0', 'deeplabv3_resnet50', pretrained=True)
model.eval()
x = torch.randn(1, 3, 224, 224, requires_grad=True)
y = model(x)
torch_out = torch.onnx._export(model,                   # model being run
                                x,                      # model input (or a tuple for multiple inputs)
                                BEST_MODEL_PATH_ONNX,   # where to save the model (can be a file or file-like object)
                                export_params=True,     # store the trained parameter weights inside the model file
                                input_names=['Conv2d0_3-64'],     # specify the name of input layer in onnx model
                                output_names=['Linear2_4096-2'])    # specify the name of output layer
print("Successfully genereated ONNX model at ",BEST_MODEL_PATH_ONNX)
```

2. Install ONNX on Ubuntu system

```
pip install onnx
```

3. Set the environment path for ONNX

```
cd $SNPE_ROOT
export ONNX_DIR="your_onnx_installation_dir"
source bin/envsetup.sh -o $ONNX_DIR
```

4. Run ONNX to DLC conversion command

```
snpe-onnx-to-dlc --input_dim sub_7 1,513,513,3 --out_node ArgMax --input_network /deeplabv3_onnx_model.onnx --output_path deeplab_pt.dlc
```

5. Once the dlc format is generated successfully, follow the "Running Inference" sections in TensorFlow section to run inference on both Ubuntu and RB5

{% image Example.png Example Input Image %}

{% image Mask.png Generated Mask %}

{% image PostProcessed.png Post Processed Image %}

