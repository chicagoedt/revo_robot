from keras.models import load_model

model = load_model('best.h5')
model.save_weights('best_weights.h5')
