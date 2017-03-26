from keras.models import load_model
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint
import numpy as np
import sys

img_height = 224
img_width = 224
img_size = (img_height, img_width)
input_shape = (img_height, img_width, 3)
batch_size = 64
epochs = 500
steps_per_epoch = int(1539/batch_size) + 1
steps_on_val = int(398/batch_size) + 1

model = load_model(sys.argv[1])
#model.compile(optimizer='adadelta', loss='categorical_crossentropy', metrics=['accuracy'])

datagen = ImageDataGenerator(
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        horizontal_flip=True,
        zoom_range=0.2,
        rescale=1./255)
val_datagen = ImageDataGenerator(rescale=1./255)

training_generator = datagen.flow_from_directory(
        'data/training',
        target_size=img_size,
        batch_size=batch_size)

validation_generator = val_datagen.flow_from_directory(
        'data/validation',
        target_size=img_size,
        batch_size=batch_size)

checkpoint = ModelCheckpoint(
        'best.h5',
        monitor='val_loss',
        verbose=0,
        save_best_only=True)

tb = TensorBoard(
        log_dir='./logs',
        histogram_freq=0,
        write_graph=True,
        write_images=True)
'''
i = 1
e = 1
print("Epoch 1/" + str(epochs))
for x_train, y_train in training_generator:
    print("Batch " + str(i) + " of " + str(steps_per_epoch) + ":")
    print(model.train_on_batch(x_train, [y_train, y_train, y_train]))
    i += 1
    if i > steps_per_epoch:
	model.save(str(e) + '.h5')
	print("Validating...")
        i = 1
        for x_val, y_val in validation_generator:
            print(model.test_on_batch(x_val, [y_val, y_val, y_val]))
            i += 1
            if i > steps_on_val:
                i = 1
                break
        print(model.metrics_names)
        e += 1
        print("Epoch " + str(e) + "/" + str(epochs))
    if e > epochs:
        break

'''
model.fit_generator(
        training_generator,
        steps_per_epoch=steps_per_epoch,
        epochs=epochs,
        callbacks=[checkpoint, tb],
        validation_data=validation_generator,
        validation_steps=steps_on_val)

