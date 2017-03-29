from keras.applications.vgg16 import VGG16
from keras.models import Sequential
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D

#vgg = VGG16(include_top=False, weights='imagenet', input_shape=input_shape)

def buildModel():
    model = Sequential()

    model.add(Conv2D(64, (3,3), padding='same', activation='relu', input_shape=(224,224,3)))
    model.add(Conv2D(64, (3,3), padding='same', activation='relu'))
    model.add(MaxPooling2D((2,2)))

    model.add(Conv2D(128, (3,3), padding='same', activation='relu'))
    model.add(Conv2D(128, (3,3), padding='same', activation='relu'))
    model.add(MaxPooling2D((2,2)))

    model.add(Conv2D(256, (3,3), padding='same', activation='relu'))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu'))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu'))
    model.add(MaxPooling2D((2,2)))

    model.add(Conv2D(512, (3,3), padding='same', activation='relu'))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu'))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu'))

    model.add(Conv2D(512, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu', dilation_rate=2))

    model.add(Conv2D(4096, (7,7), padding='same', dilation_rate=4))
    model.add(Dropout(0.5))
    model.add(Conv2D(4096, (1,1), padding='same', activation='relu'))
    model.add(Dropout(0.5))
    model.add(Conv2D(2, (1,1), padding='same'))

    model.compile(optimizer='adadelta', loss='categorical_crossentropy', metrics=['accuracy'])

    plot_model(model, 'dilated.png', show_shapes=True)

    return model

#TODO: load VGG weights (save to h5 and import by_name), create generators, val generator resizes images to 28x28
