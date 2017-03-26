from keras.models import Model, load_model
from keras.applications.vgg16 import VGG16
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint, EarlyStopping
from keras.layers import Flatten, Dropout, Dense, Conv2D
from keras.utils import plot_model
import numpy as np
import sys, string

# HYPERPARAMETERS
img_height = 224
img_width = 224
img_size = (img_height, img_width)
input_shape = (img_height, img_width, 3)
batch_size = 2
epochs = 500
steps_per_epoch = int(1539/batch_size) + 1
steps_on_val = int(398/batch_size) + 1

# If the user specifies a model as a starting point, load it, otherwise use vanilla VGG16.
if len(sys.argv) > 1:
    vgg = load_model(sys.argv[1])
else:
    vgg = VGG16(weights='imagenet', include_top=False, input_shape=input_shape)

# Create generators and callbacks.
datagen = ImageDataGenerator(
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        horizontal_flip=True,
        zoom_range=0.2,
        rescale=1./255)
training_generator = datagen.flow_from_directory(
        'data/training',
        target_size=img_size,
        batch_size=batch_size)

validation_generator = ImageDataGenerator(rescale=1./255).flow_from_directory(
        'data/validation',
        target_size=img_size,
        batch_size=batch_size)

tb = TensorBoard(
        log_dir='./logs',
        histogram_freq=0,
        write_graph=True,
        write_images=True)

earlystop = EarlyStopping(patience=10)

def createClassificationBlock(input, prefix, kernel, flatten):
    conv1 = Conv2D(512, kernel, activation='relu', name=prefix+'conv1')(input)
    conv2 = Conv2D(512, (1,1), activation='relu', name=prefix+'conv2')(conv1)
    pred = Conv2D(3, (1,1), activation='softmax', name=prefix+'pred')(conv2)
    out = Flatten()(pred)

    if flatten:
        return out
    else:
        return pred

# Create a model which classifies after three blocks of convolutions.
def createNBlockModel(n):
    if n == 3:
        prefix = 'pool3_'
        weight_save_name = 'three_block_weights.h5'
        model_save_name = 'three_block_model.h5'
        kernel = (28,28)
    elif n == 4:
        prefix = 'pool4_'
        weight_save_name = 'four_block_weights.h5'
        weight_load_name = 'three_block_weights.h5'
        model_save_name = 'four_block_model.h5'
        kernel = (14,14)
    elif n == 5:
        prefix = 'pool5_'
        weight_save_name = 'five_block_weights.h5'
        weight_load_name = 'four_block_weights.h5'
        model_save_name = 'five_block_model.h5'
        kernel = (7,7)

    for layer in vgg.layers:
        if layer.name == 'block' + str(n) + '_pool':
            pool_out = layer.output
            break

    out = createClassificationBlock(input=pool_out, prefix=prefix, kernel=kernel, flatten=True)
    model = Model(vgg.input, out)

    # Load weights from the previous training session. Freeze them to accelerate training time.
    if n !=3:
        model.load_weights('active_models/' + weight_load_name, by_name=True)
        for layer in model.layers:
            if layer.name.find(prefix) != -1:
                break
            layer.trainable=False

    # Train the model until it stops improving; save the result.
    model.compile(optimizer='adadelta', loss='categorical_crossentropy', metrics=['accuracy'])
    model.fit_generator(
            training_generator,
            steps_per_epoch=steps_per_epoch,
            epochs=epochs,
            callbacks=[tb, earlystop],
            validation_data=validation_generator,
            validation_steps=steps_on_val)
    model.save_model('active_models/' + model_save_name)
    model.save_weights('active_models/' + weight_save_name)

createNBlockModel(3)
createNBlockModel(4)
createNBlockModel(5)

m5 = load_model('active_models/five_block_model.h5')
for layer in m5.layers:
    if layer.name == 'block3_pool':
        block3_out = layer.output
    elif layer.name == 'block4_pool':
        block4_out = layer.output

FCN_16s = createClassificationBlock(input=block3_out, prefix='pool3_', kernel=(28,28), flatten=True)
FCN_8s = createClassificationBlock(input=block4_out, prefix='pool4_', kernel=(14,14), flatten=True)

FCN = Model(vgg.input, [m5.output, FCN_16s, FCN_8s])
FCN.load_weights('active_models/three_block_weights.h5', by_name=True)
FCN.load_weights('active_models/four_block_weights.h5', by_name=True)
FCN.load_weights('active_models/five_block_weights.h5', by_name=True)

FCN.save_model('active_models/FCN.h5')
plot_model(FCN, 'FCN.png', show_shapes=True)
print("Done!")
