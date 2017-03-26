from keras.models import Model, load_model
from keras.preprocessing.image import ImageDataGenerator
from keras.applications.vgg16 import VGG16
from keras.layers import Input, Conv2D, UpSampling2D, Flatten
import numpy as np
import cv2
import sys, os, string, random


cnn = load_model(sys.argv[1])
weights = cnn.get_weights()
vgg = VGG16(include_top=False, weights='imagenet', input_shape=(224,224,3))
fc6_weights = cnn.layers[20].get_weights()
conv6_weights = [fc6_weights[0].reshape([7,7,512,1024]), fc6_weights[1]]
fc7_weights = cnn.layers[22].get_weights()
conv7_weights = [fc7_weights[0].reshape([1,1,1024,1024]), fc7_weights[1]]
fc_pred_weights = cnn.layers[24].get_weights()
conv_pred_weights = [fc_pred_weights[0].reshape([1,1,1024,3]), fc_pred_weights[1]]

i = 0
for layer in cnn.layers:
    print(layer.name + ': ' + str(i))
    i += 1

pool5 = vgg.layers[18].output
conv6 = Conv2D(1024, (7,7), activation='relu', padding = 'valid')(pool5)
conv7 = Conv2D(1024, (1,1), activation='relu', padding = 'valid')(conv6)
preds = Conv2D(3, (1,1), activation='softmax')(conv7)
#out = UpSampling2D((32,32))(preds)
out = Flatten()(preds)

fcn = Model(vgg.input, preds)

'''
fcn.layers[19].set_weights([cnn.layers[20].get_weights()[0].reshape([7,7,512,1024])])
fcn.layers[20].set_weights([weights[22].reshape([1,1,1024,1024])])
fcn.layers[21].set_weights([weights[24].reshape([1,1,1024,3])])
'''

fcn.layers[11].set_weights(cnn.layers[11].get_weights())
fcn.layers[12].set_weights(cnn.layers[12].get_weights())
fcn.layers[13].set_weights(cnn.layers[13].get_weights())
fcn.layers[15].set_weights(cnn.layers[15].get_weights())
fcn.layers[16].set_weights(cnn.layers[16].get_weights())
fcn.layers[17].set_weights(cnn.layers[17].get_weights())
fcn.layers[19].set_weights(conv6_weights)
fcn.layers[20].set_weights(conv7_weights)
fcn.layers[21].set_weights(conv_pred_weights)

fcn.compile(optimizer='adagrad', loss='categorical_crossentropy', metrics=['accuracy'])

'''
datagen = ImageDataGenerator(rescale=1./255)
validation_generator = datagen.flow_from_directory(
        'data/validation',
        target_size=(224,224),
        batch_size=2)

print(fcn.predict_generator(validation_generator, 7)[0])
'''



img = cv2.imread('test.png')
height, width = img.shape[:2]
inp = img.reshape(1,height,width,3)
outp = fcn.predict(inp)
print(outp)
