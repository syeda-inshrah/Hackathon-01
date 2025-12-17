---
title: "Week 2 - Voice Command Integration with OpenAI Whisper"
sidebar_label: "Week 2: Voice Commands with Whisper"
description: "Understanding OpenAI Whisper integration for voice command processing in robotics"
keywords: [whisper, voice, command, speech, recognition, robotics, ai]
---

# Week 2: Voice Command Integration with OpenAI Whisper

## Learning Outcomes

By the end of this week, you will be able to:
- Integrate OpenAI Whisper for speech-to-text conversion in robotics
- Process and filter voice commands for robotic applications
- Handle noise and speech recognition errors in robot environments
- Implement voice command parsing for robotic task execution
- Design robust voice interfaces for humanoid robots

## Introduction to OpenAI Whisper

OpenAI Whisper is a state-of-the-art speech recognition system that converts speech into text with high accuracy across multiple languages. Unlike traditional speech recognition systems that are limited to specific domains or languages, Whisper has been trained on 680,000 hours of multilingual and multitask supervised data, making it robust for diverse applications.

### Key Features of Whisper

- **Multilingual Support**: Supports 99+ languages
- **Robustness**: Performs well on noisy audio
- **Task Versatility**: Can perform speech recognition, translation, and language identification
- **Large-Scale Training**: Trained on diverse, web-scraped audio-text pairs

### Whisper Models

- **tiny**: Fastest, least accurate (76MB)
- **base**: Good balance of speed and accuracy (145MB)
- **small**: Better accuracy (484MB)
- **medium**: Higher accuracy (1.5GB)
- **large**: Best accuracy (3.1GB)

## Voice Command Processing Pipeline

### Audio Capture and Preprocessing

For robotics applications, the audio capture pipeline needs to handle various challenges:

- **Background Noise**: Robot motors, fans, and environmental sounds
- **Distance**: Speaker may be at varying distances from microphone
- **Directionality**: Microphones may need to focus on specific directions
- **Real-time Requirements**: Processing with minimal latency

```python title="Audio Processing for Voice Commands"
import numpy as np
import pyaudio
import webrtcvad
from scipy import signal
import queue
import threading

class AudioProcessor:
    def __init__(self, sample_rate=16000, frame_duration=30):
        self.sample_rate = sample_rate
        self.frame_duration = frame_duration  # in ms
        self.frame_size = int(sample_rate * frame_duration / 1000)
        
        # Voice Activity Detection (VAD) to detect speech segments
        self.vad = webrtcvad.Vad(2)  # Aggressiveness level 2
        
        # Audio buffer for processing
        self.audio_buffer = queue.Queue()
        self.recording = False
        
        # For noise reduction
        self.noise_buffer = np.zeros(10000)  # Buffer to store noise samples
        self.noise_buffer_idx = 0
        self.noise_buffer_full = False
        
    def start_listening(self):
        """Start audio capture"""
        self.recording = True
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Open audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.frame_size
        )
        
        # Start capture thread
        self.capture_thread = threading.Thread(target=self.capture_audio)
        self.capture_thread.start()
    
    def capture_audio(self):
        """Capture audio in a separate thread"""
        while self.recording:
            # Read audio data
            data = self.stream.read(self.frame_size, exception_on_overflow=False)
            audio_frame = np.frombuffer(data, dtype=np.int16)
            
            # Apply noise reduction if needed
            processed_frame = self.apply_noise_reduction(audio_frame)
            
            # Add to buffer if voice is detected
            if self.is_voice_present(processed_frame):
                self.audio_buffer.put(processed_frame)
    
    def is_voice_present(self, audio_frame):
        """Check if voice is present in the audio frame using VAD"""
        # Convert to bytes for VAD (16-bit, mono, 16kHz)
        audio_bytes = audio_frame.astype(np.int16).tobytes()
        
        # Check for voice activity
        try:
            return self.vad.is_speech(audio_bytes, self.sample_rate)
        except:
            return False
    
    def apply_noise_reduction(self, audio_frame):
        """Apply simple noise reduction by subtracting estimated noise"""
        # For this example, we'll use spectral subtraction approach
        # In practice, more sophisticated techniques like Wiener filtering would be used
        
        # Update noise estimate (this is a simplified approach)
        if self.noise_buffer_full:
            noise_estimate = np.mean(self.noise_buffer)
            reduced_frame = audio_frame - noise_estimate
        else:
            # Build noise buffer from initial frames
            start_idx = self.noise_buffer_idx
            end_idx = min(start_idx + len(audio_frame), len(self.noise_buffer))
            self.noise_buffer[start_idx:end_idx] = audio_frame[:end_idx - start_idx]
            
            self.noise_buffer_idx += len(audio_frame)
            if self.noise_buffer_idx >= len(self.noise_buffer):
                self.noise_buffer_idx = 0
                self.noise_buffer_full = True
            
            reduced_frame = audio_frame
        
        # Apply a simple high-pass filter to remove low frequency noise
        b, a = signal.butter(3, 0.05, btype='high')
        reduced_frame = signal.filtfilt(b, a, reduced_frame)
        
        # Clip to avoid overflow
        reduced_frame = np.clip(reduced_frame, -32768, 32767)
        return reduced_frame.astype(np.int16)
    
    def get_audio_segment(self):
        """Get a complete audio segment for processing"""
        if self.audio_buffer.empty():
            return None
        
        # Collect frames until a period of silence
        segment = []
        silence_counter = 0
        
        while not self.audio_buffer.empty() and silence_counter < 10:
            frame = self.audio_buffer.get()
            segment.append(frame)
            
            # Check if this frame contains voice activity
            if not self.is_voice_present(frame):
                silence_counter += 1
            else:
                silence_counter = 0
        
        if len(segment) > 0:
            return np.concatenate(segment)
        return None
    
    def stop(self):
        """Stop audio capture"""
        self.recording = False
        
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        
        if hasattr(self, 'audio'):
            self.audio.terminate()

# Example usage
processor = AudioProcessor()
processor.start_listening()

# Simulate getting an audio segment (in practice, this would happen continuously)
import time
time.sleep(2)  # Let it capture some audio

audio_segment = processor.get_audio_segment()
if audio_segment is not None:
    print(f"Captured audio segment of {len(audio_segment)} samples")
else:
    print("No audio segment available")

processor.stop()
```

## Whisper Integration for Voice Commands

### Installing Whisper

```bash
pip install openai-whisper
```

### Using Whisper for Voice Command Recognition

```python title="Whisper Integration Example"
import whisper
import numpy as np
import librosa
import torch

class WhisperVoiceCommandProcessor:
    def __init__(self, model_size="base", device="cpu"):
        self.device = device
        
        # Load Whisper model - this will download the model if not already present
        self.model = whisper.load_model(model_size, device=device)
        
        # Robot-specific command vocabulary
        self.robot_commands = {
            "move forward", "move backward", "turn left", "turn right",
            "stop", "go", "pick up", "grasp", "release", "place",
            "bring me", "take", "go to", "move to", "clean",
            "come here", "follow me", "wait", "help"
        }
    
    def process_audio_with_whisper(self, audio):
        """
        Process audio with Whisper and return transcribed text
        """
        # Convert audio to 16kHz if needed
        if isinstance(audio, np.ndarray):
            # If audio is numpy array, it's likely already at the right format
            if len(audio.shape) > 1:
                # Convert stereo to mono
                audio = np.mean(audio, axis=1)
        else:
            # If audio is a file path, load it
            audio, sr = librosa.load(audio, sr=16000)
        
        # Run Whisper to transcribe audio
        result = self.model.transcribe(audio, fp16=False if self.device == "cpu" else True)
        transcribed_text = result["text"].strip()
        
        return transcribed_text
    
    def parse_robot_command(self, text):
        """
        Parse the transcribed text to identify robot commands
        """
        text_lower = text.lower()
        
        # Find if any command is present in the text
        detected_commands = []
        for cmd in self.robot_commands:
            if cmd in text_lower:
                detected_commands.append(cmd)
        
        # Extract action and target if present
        action = None
        target = None
        
        # Simple action extraction (in practice, this would be more sophisticated)
        if any(cmd in text_lower for cmd in ["pick up", "grasp", "take"]):
            action = "grasp"
        elif any(cmd in text_lower for cmd in ["place", "put"]):
            action = "place"
        elif any(cmd in text_lower for cmd in ["move", "go", "come"]):
            action = "navigate"
        elif any(cmd in text_lower for cmd in ["stop", "wait"]):
            action = "stop"
        elif any(cmd in text_lower for cmd in ["clean"]):
            action = "clean"
        
        # Extract target object if present
        # This is a simplified approach - in practice, you'd use more advanced NLP
        possible_targets = ["cup", "bottle", "book", "box", "table", "chair", "ball", "phone"]
        for obj in possible_targets:
            if obj in text_lower:
                target = obj
                break
        
        return {
            'raw_text': text,
            'detected_commands': detected_commands,
            'action': action,
            'target': target,
            'confidence': 0.9  # Placeholder - would come from model in practice
        }
    
    def process_voice_command(self, audio_input):
        """
        Complete pipeline: audio input → Whisper transcription → command parsing
        """
        # Transcribe audio
        transcribed_text = self.process_audio_with_whisper(audio_input)
        
        # Parse command
        command_info = self.parse_robot_command(transcribed_text)
        
        return command_info

# Example usage (with dummy audio)
# In practice, you would pass actual audio data
processor = WhisperVoiceCommandProcessor(model_size="base")
print("Whisper model loaded")

# Example of parsing a transcribed command
example_text = "Please pick up the red cup from the table"
parsed_command = processor.parse_robot_command(example_text)
print(f"Parsed command: {parsed_command}")
```

## Integration with ROS 2

### Voice Command Node Implementation

```python title="ROS 2 Voice Command Node"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger
import numpy as np
import pyaudio
import threading
import time

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Initialize Whisper processor
        # Note: In practice, Whisper initialization would be done here
        # For this example, we'll simulate it
        self.whisper_processor = self.initialize_whisper()
        
        # Audio processor
        self.audio_processor = AudioProcessor()
        
        # Robot command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/voice_command_status', 10)
        
        # Service to start/stop listening
        self.start_listening_srv = self.create_service(Trigger, 'start_listening', self.start_listening_callback)
        self.stop_listening_srv = self.create_service(Trigger, 'stop_listening', self.stop_listening_callback)
        
        # Initialize state
        self.listening = False
        self.listening_thread = None
        
        self.get_logger().info('Voice Command Node Initialized')
    
    def initialize_whisper(self):
        """Initialize Whisper model (simulated for this example)"""
        # In practice: return WhisperVoiceCommandProcessor()
        class MockWhisper:
            def process_voice_command(self, audio_input):
                # Simulated transcription
                return {
                    'raw_text': 'move forward',
                    'detected_commands': ['move forward'],
                    'action': 'navigate',
                    'target': None,
                    'confidence': 0.9
                }
        return MockWhisper()
    
    def start_listening_callback(self, request, response):
        """Start voice command recognition"""
        if not self.listening:
            self.start_voice_recognition()
            response.success = True
            response.message = 'Started listening for voice commands'
        else:
            response.success = False
            response.message = 'Already listening'
        
        self.publish_status(f"Listening status: {response.message}")
        return response
    
    def stop_listening_callback(self, request, response):
        """Stop voice command recognition"""
        if self.listening:
            self.stop_voice_recognition()
            response.success = True
            response.message = 'Stopped listening for voice commands'
        else:
            response.success = False
            response.message = 'Not currently listening'
        
        self.publish_status(f"Listening status: {response.message}")
        return response
    
    def start_voice_recognition(self):
        """Start the voice recognition process"""
        if not self.listening:
            self.listening = True
            self.get_logger().info('Starting voice recognition')
            
            # Start audio processor
            self.audio_processor.start_listening()
            
            # Start recognition thread
            self.listening_thread = threading.Thread(target=self.voice_recognition_loop)
            self.listening_thread.start()
    
    def stop_voice_recognition(self):
        """Stop the voice recognition process"""
        if self.listening:
            self.listening = False
            self.get_logger().info('Stopping voice recognition')
            
            # Stop audio processor
            self.audio_processor.stop()
            
            if self.listening_thread:
                self.listening_thread.join()
    
    def voice_recognition_loop(self):
        """Main loop for voice recognition"""
        while self.listening:
            # Get audio segment
            audio_segment = self.audio_processor.get_audio_segment()
            
            if audio_segment is not None and len(audio_segment) > 0:
                # Process with Whisper
                try:
                    command_info = self.whisper_processor.process_voice_command(audio_segment)
                    self.get_logger().info(f'Recognized: {command_info["raw_text"]}')
                    
                    # Execute the command
                    self.execute_robot_command(command_info)
                    
                except Exception as e:
                    self.get_logger().error(f'Error processing voice command: {str(e)}')
            
            # Small delay to prevent busy waiting
            time.sleep(0.1)
    
    def execute_robot_command(self, command_info):
        """Execute the robot command based on voice input"""
        action = command_info.get('action')
        target = command_info.get('target')
        
        if action is None:
            self.get_logger().info('No recognized action in command')
            return
        
        # Create a Twist message for robot movement
        twist_msg = Twist()
        
        if action == 'navigate':
            # Simple forward movement for demonstration
            twist_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            self.get_logger().info('Moving forward')
        elif action == 'grasp':
            # In a real system, this would trigger arm manipulation
            self.get_logger().info(f'Attempting to grasp {target or "object"}')
        elif action == 'place':
            # In a real system, this would trigger placing action
            self.get_logger().info(f'Attempting to place {target or "object"}')
        elif action == 'stop':
            # Stop movement
            self.get_logger().info('Stopping robot')
        
        # Publish command
        self.cmd_vel_pub.publish(twist_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = f"Executed: {command_info['raw_text']} -> {action} {target or ''}"
        self.status_pub.publish(status_msg)
    
    def publish_status(self, message):
        """Publish status message"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Speech Recognition Errors

### Confidence Thresholds and Validation

Whisper provides confidence estimates for its transcriptions. In robotic applications, it's important to validate commands before execution:

```python title="Voice Command Validation"
class VoiceCommandValidator:
    def __init__(self):
        self.robot_commands = {
            "move forward", "move backward", "turn left", "turn right",
            "stop", "go", "pick up", "grasp", "release", "place",
            "bring me", "take", "go to", "move to", "clean",
            "come here", "follow me", "wait", "help"
        }
        
        self.confidence_threshold = 0.7
        self.robot_name_keywords = ["robot", "assistant", "help", "hey"]
    
    def validate_command(self, command_info):
        """
        Validate if the voice command is appropriate for robot execution
        """
        text = command_info.get('raw_text', '').lower()
        confidence = command_info.get('confidence', 0)
        
        # Check confidence
        if confidence < self.confidence_threshold:
            return False, f"Low confidence ({confidence:.2f} < {self.confidence_threshold})"
        
        # Check if the command is a robot command
        has_robot_command = any(cmd in text for cmd in self.robot_commands)
        if not has_robot_command:
            return False, f"No recognized robot command in: {text}"
        
        # Check if the command seems directed at the robot
        is_directed_at_robot = any(keyword in text for keyword in self.robot_name_keywords)
        
        # If confidence is high but not explicitly directed at robot, still accept
        if confidence > 0.85:
            return True, f"High confidence command: {text}"
        
        # Otherwise, require it to be directed at the robot
        if is_directed_at_robot:
            return True, f"Command directed at robot: {text}"
        
        return False, f"Command not directed at robot: {text}"
    
    def suggest_correction(self, text):
        """
        Suggest possible corrections for unclear commands
        """
        # This is a simplified approach
        # In practice, you'd use more sophisticated NLP techniques
        suggestions = []
        text_lower = text.lower()
        
        # Check for common misrecognitions
        if 'left' in text_lower and 'right' not in text_lower:
            suggestions.append("turn left")
        elif 'right' in text_lower and 'left' not in text_lower:
            suggestions.append("turn right")
        
        if 'go' in text_lower or 'move' in text_lower:
            suggestions.append("move forward")
            suggestions.append("move backward")
        
        return suggestions

# Example usage
validator = VoiceCommandValidator()

# Test command validation
test_commands = [
    {"raw_text": "move forward please", "confidence": 0.85},
    {"raw_text": "what's the weather like", "confidence": 0.9},
    {"raw_text": "robot please stop", "confidence": 0.75},
    {"raw_text": "pick up the cup", "confidence": 0.6}
]

for cmd in test_commands:
    is_valid, message = validator.validate_command(cmd)
    print(f"Command: '{cmd['raw_text']}' - Valid: {is_valid}, {message}")
    
    if not is_valid:
        suggestions = validator.suggest_correction(cmd['raw_text'])
        if suggestions:
            print(f"  Suggestions: {suggestions}")
    print()
```

## Real-World Considerations

### Audio Quality in Robot Environments

Robots often operate in noisy environments, which can degrade speech recognition performance. Consider:

- **Microphone placement**: Position microphones for optimal voice capture
- **Active noise cancellation**: Use hardware or software to reduce robot noise
- **Beamforming**: Use multiple microphones to focus on speaker direction
- **Distance management**: Implement voice activation even when speaker is distant

### Privacy and Security

When implementing voice recognition systems:

- **Data handling**: Ensure voice data is processed and stored appropriately
- **Local processing**: Consider processing voice commands locally to preserve privacy
- **Authentication**: Implement voice authentication for sensitive commands
- **Audit trails**: Log command execution for debugging and safety

## Hands-On Exercise

1. Install OpenAI Whisper and experiment with different model sizes
2. Set up audio capture with noise reduction in a simulated environment
3. Create a simple voice command system that can recognize and execute basic robot commands
4. Implement command validation to filter inappropriate commands
5. Test the system with various audio conditions (noisy, quiet, distant)

## Summary

Integrating OpenAI Whisper into robotic systems enables natural voice interaction with humanoid robots. The process involves audio capture and preprocessing, speech recognition with Whisper, and command parsing for robot execution. Key challenges include managing audio quality in robotic environments, validating commands before execution, and ensuring privacy and security. With proper implementation, voice command interfaces can significantly enhance the usability and accessibility of humanoid robots.