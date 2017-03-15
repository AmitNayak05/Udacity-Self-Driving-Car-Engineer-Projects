Model Architecture Design:
VGG16 model and weights trained on imagenet was used as the base model for 
training on the dataset.

It was selected as using a pretrained model would greatly redce the training time.
The VGG16 being a highly accurate model was my choice for the project.

The first layer is a input layer, which receives an input image, and pads with zeroes.
Then is the Convolution layer with 64 convolution filters with a 3x3 kernel and relu 
activation. It is again padded with zeroes and passed on to a similar convolution 
layer of 64-3x3 convolution kernels. Then there is a maxpooling layer of 2x2 window.

The output of pooling layer is padded with zeroes and is passed on to a convolution
layer with 128-3x3 convolution kernels and relu activation, followe by a maxpooling 
and a similar convolution layer.

Similarly, the output is passed to three convolution layers of 256-3x3 convolution kernels
preceeded by zero padding. A maxpooling layer follows. Two similar structures follow but they 
consist of 512-3x3 convolution kernels.  

On top of it, I add  a globalaveragepooling layer followed a dropout layer with
50% dropout rate. Dropout used in the model helps in regularization and prevents 
overfitting. After these are 2 fully connected layers with 1024 and 512 
neurons and having ELU activtion for smooth angle transition.

This was followed by a logistic layer which has linear activation.


Architecture Characteristics:
The initial layers give us basic features like color and direction, and with higher layers
they combine to form complex patterns. So training them will modilfy them to detect features
from the images of track.

Training the model completely helps finetune weights to better detect basic features from 
track. After some epochs, if we train only the later layers of the model, it would greatly 
improve the model prediction.  



Training Data:
For training, the dataset provided by Udacity team is used.
The images used are:

1) Center Images:- The center images are stored as training data with corresponding 
steering in the sheet being the desired steering angle for the frame.

2) Left Images:- The left images are used to generate more data for training. The left images
represent a scenario when the vehicle has a left offset and hence we need to modify the 
desired steering angle for image as being inclined to turn right by some angle as
compared to sterring with with respect to the center frame. For this, we add a value of
0.25 to the steering angle of the vehicle

3) Right images: The right images are used to generate more data for training. The right
images represent a scenario when the vehicle has a right offset and hence we need to 
modify the desired steering angle for image as being inclined to turn left by some 
angle as compared to sterring with with respect to the center frame. For this, we 
substract a value of 0.25 from the steering angle of the vehicle.

4) Flipped Images: All the images including the center, left and right images are
flipped and added to the training dataset. The corresponding desired steering angles
are inferred as the negetive of the steering angle of original dimage. This doubles
the training data.



Data Preprocessing:
1) We do some image rocessing to make our training data more efficient. First, we 
normalize images using normallize function of Opencv to normalize the image to
range(0,1).

2)The the normalized images are cropped in order to remove the horizon from the image,
as they are not essential for training. The horizon is at the top of the image covering 
around 30% of the image from top. So by cropping we remove this unrequired information.
After cropping we are left with image i which main characteristics are road and 
lane markings.

3)The cropped images are the resized to a smaller size of 64x64 for faster training
and feature extraction.



Model Training:
The dataset was divided into 2 sets namely training and validation data. Around 5% of data
was taken for validation set. The model was trainedeach epoch and loss on validation 
set was computed. The training was stopped when the validation accuracy was 
higher than the last epoch.
The model was compiled using Adam optimizer with learning rate of 0.00001. All the 
layers are trained for 7 epochs in order to get the initial appropriate weights for
all the layers. 
After the weights are set, we freeze the weights of the initial 15 layers and 
train the last 8 layers for another 6 epochs to get proper weights of the those layers.
The training loss was less than 0.5.
In order to determine the number of epochs to train, the validation loss was observed.
Also, model and weights were saved after each epoch under different names to evaluate performance
after each epoch.



Testing(drive.py):
For testing the model, the drive.py file was modified to process the images before using 
the model to predict the steering angle from the image. 
The input image are preproceesed by:
1) Normalize the imagesto range (0-1), as the model has been trained on normalized images
2) Crop image to remove horizon
The image is then used for predicting steering using the model.



Result:
The car was able to navigate the test track on the left under different resolutions 
of the simulator. I checked the working of model for 10 loops and it was able to
remain in track. 



Last Submitted Model:
The model I submitted in last assignment worked when I ran the simulator 
for the resolution 320x240. I had not checked its working in different 
resolutions of simulator and on rechecking I found that it fails
for higher resolution of simulator suchas 800x600.

Reference:
https://keras.io/applications/