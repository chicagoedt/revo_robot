from keras.models import Model
from keras.applications.vgg16 import VGG16
from keras.layers import Flatten, Dropout, Dense, Conv2D
from keras.utils import plot_model
import string, sys

vgg = VGG16(weights='imagenet', include_top=False, input_shape=(224,224,3))
for layer in vgg.layers:
    layer.trainable = False
    if layer.name == 'block3_pool':
        break

flat = Flatten()(vgg.output)
conv6 = Conv2D(1024, (7,7), activation='relu', name='conv6')(vgg.output)
drop1 = Dropout(0.5)(conv6)
conv7 = Conv2D(1024, (1,1), activation='relu', name='conv7')(drop1)
drop2 = Dropout(0.5)(conv7)
conv7pred = Flatten()(Conv2D(3, (1,1), activation='softmax', name='classify')(drop2))
#pool4pred = Flatten()(Conv2D(3, (14,14), activation='softmax', name='pool4pred')(vgg.layers[14].output))
#pool3pred = Flatten()(Conv2D(3, (28,28), activation='softmax', name='pool3pred')(vgg.layers[10].output))

model = Model(inputs=[vgg.input],outputs=[conv7pred])
model.compile(optimizer='adadelta', loss='categorical_crossentropy', metrics=['accuracy'])
model.save(sys.argv[1] + '.h5')
plot_model(model, sys.argv[1] + '.png', show_shapes=True)
