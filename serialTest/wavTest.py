import wave

with wave.open("output.wav") as wav_file:
    metadata = wav_file.getparams()
    frames = wav_file.readframes(metadata.nframes)

    print(metadata)