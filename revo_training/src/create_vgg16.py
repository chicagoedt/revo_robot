from keras.models import Model
from keras.applications.vgg16 import VGG16
from keras.layers import Flatten, Dropout, Dense
from keras.utils import plot_model
import string, sys

vgg = VGG16(weights='imagenet', include_top=False, input_shape=(224,224,3))
for layer in vgg.layers:
    layer.trainable = False
    if layer.name == 'block3_pool':
        break

flat = Flatten()(vgg.output)
fc6 = Dense(1024, activation='relu', name='fc6')(flat)
drop1 = Dropout(0.5)(fc6)
fc7 = Dense(1024, activation='relu', name='fc7')(drop1)
drop2 = Dropout(0.5)(fc7)
fc7pred = Dense(3, activation='softmax', name='classify')(drop2)
pool4pred = Dense(3, activation='softmax', name='pool4pred')(Flatten()(vgg.layers[14].output))
pool3pred = Dense(3, activation='softmax', name='pool3pred')(Flatten()(vgg.layers[10].output))

model = Model(inputs=[vgg.input],outputs=[fc7pred,pool4pred,pool3pred])
model.compile(optimizer='adadelta', loss='categorical_crossentropy', metrics=['accuracy'], loss_weights=[1,1,1])
model.save(sys.argv[1] + '.h5')
plot_model(model, sys.argv[1] + '.png', show_shapes=True)
