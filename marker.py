import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt  # for plotting (optional)

def create_sync_sequence(sync_word, sample_rate, raw_bit_rate):
    # Convert sync word to binary string
    binary_sync_word = bin(sync_word)[2:].zfill(32)

    # Create the bit sequence
    bit_sequence = [int(bit) for bit in binary_sync_word]

    # Upsample the bit sequence to match the audio sampling rate
    samples_per_bit = int(sample_rate / raw_bit_rate)
    upsampled_sequence = np.repeat(bit_sequence, samples_per_bit)

    return upsampled_sequence

def detect_headers(wav_file, sync_word, header_length, cyclic_header_interval):
    # Read the WAV file
    sample_rate, audio_data = wavfile.read(wav_file)


# Parameters
    raw_bit_rate = 2500  # bit/s
    samples_per_bit = int(sample_rate / raw_bit_rate)
    header_interval_samples = cyclic_header_interval * samples_per_bit

    # Create the sync sequence
    sync_sequence = create_sync_sequence(sync_word, sample_rate, raw_bit_rate)

    # Find headers within intervals
    headers = []
    for i in range(0, len(audio_data) - header_interval_samples, header_interval_samples):
        interval_data = audio_data[i:i+header_interval_samples]
        correlation_result = np.correlate(interval_data, sync_sequence, mode='valid')
        synchronization_position = np.argmax(correlation_result)
        header_position = i + synchronization_position
        headers.append(header_position)


    # Plotting cross-correlation results for each interval (optional)
    #for i, header_position in enumerate(headers):
    #    plt.plot(np.arange(header_position, header_position + len(sync_sequence)), audio_data[header_position:header_position+len(sync_sequence)])
    #    plt.title(f'Interval {i+1} - Cross-Correlation Result')
    #    plt.show()

    return headers


def mark_headers(wav_file, header_positions, mark_value=30000):
    # Read the WAV file
    sample_rate, audio_data = wavfile.read(wav_file)

    # Mark headers in the audio data
    for position in header_positions:
        print(audio_data[position])
        audio_data[position] = mark_value

    # Save the marked audio data to a new WAV file
    marked_wav_file = wav_file.replace('.wav', '_marked.wav')
    wavfile.write(marked_wav_file, sample_rate, audio_data)

if __name__ == "__main__":
    wav_file_path = "/Users/hansr/Documents/gqrx_20240120_083217_403487400_50k.wav"
    sync_word = 0x6566A5AA
    header_length = 4  # 4 bytes
    cyclic_header_interval = 560  # 560 bits
    threshold = 5000  # Adjust as needed

    header_positions = detect_headers(wav_file_path, sync_word, header_length, cyclic_header_interval)

    if len(header_positions)>0:
        print(f"Detected headers at positions: {header_positions}")
        mark_headers(wav_file_path, header_positions)
        print(f"Marked headers in the new WAV file.")
    else:
        print("No headers detected in the provided WAV file.")

