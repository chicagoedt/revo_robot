from keras.preprocessing.image import ImageDataGenerator
import numpy as np

seed=1

def format_gen_outputs(gen1,gen2):
    x1 = gen1
    y1 = gen2
    return x1, y1

data_gen_args = dict(rotation_range=30.,
                     width_shift_range=0.2,
                     height_shift_range=0.2,
                     zoom_range=0.2,
		     fill_mode='constant',
                     horizontal_flip=True,
                     rescale=1./255)

def makeTrainingGenerator(img_size, mask_size, batch_size):
    rgb_datagen = ImageDataGenerator(**data_gen_args)
    mask_datagen = ImageDataGenerator(**data_gen_args)

    rgb_generator = rgb_datagen.flow_from_directory(
            'data/segmentation/training/images/',
            target_size=img_size,
            batch_size=batch_size,
            class_mode=None,
            seed=seed)
    mask_generator = mask_datagen.flow_from_directory(
            'data/segmentation/training/masks/',
            target_size=mask_size,
            color_mode='grayscale',
            batch_size=batch_size,
            class_mode=None,
            seed=seed)

    train_generator = map(format_gen_outputs, rgb_generator, mask_generator)

    return train_generator

def makeValidationGenerator(img_size, mask_size, batch_size):
    val_rgb_datagen = ImageDataGenerator(rescale=1./255)
    val_hsv_datagen = ImageDataGenerator(rescale=1./255)
    val_otsu_datagen = ImageDataGenerator(rescale=1./255)
    val_mask_datagen = ImageDataGenerator(rescale=1./255)

    val_rgb_generator = val_rgb_datagen.flow_from_directory(
            'data/segmentation/validation/images/',
            target_size=img_size,
            batch_size=batch_size,
            class_mode=None,
            seed=seed)
    val_mask_generator = val_mask_datagen.flow_from_directory(
            'data/segmentation/validation/masks/',
            target_size=mask_size,
            color_mode='grayscale',
            batch_size=batch_size,
            class_mode=None,
            seed=seed)

    val_generator = map(format_gen_outputs, val_rgb_generator, val_mask_generator)

    return val_generator
