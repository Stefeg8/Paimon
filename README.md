# Paimon
For our team, we propose a project in which we develop a drone with voice, transcription, and image detection capabilities. Through the utilization of already established models (YOLOv10, Mistral 7B, llama 3.2 1b or 3b, OpenAI whisper, CoquiTTS) readily made available, we plan on making the drone follow a specific person, being able take voice commands, and to converse with the user through the use of a LLM. We intend to fine tune the LLM so that it can provide responses resembling that of a video game character and to also create a shell/costume for the drone so that it also resembles the character.

This repository provides the relevant code for our project.

# Specifications
We use a Mistral 7B finetune to power our drone's responses to the user. This is complemented by CoquiTTS xtts_v2 voice cloning, allowing our drone to have a voice that mimics that of Paimon's from Genshin Impact. Through OpenAI's whisper, our drone can understand human speech and take that as input for the Mistral model. 

Using YOLOv10, we achieve vision capabilities for our drone, allowing it to safely navigate its surroundings. 

# Finetuning
To finetune the model, we scraped the Genshin wiki for information about characters and the lore behind the game. By training the model on different parts of the game, we were able to give it a complete comprehension of how to give advice like Paimon. 

We processed our data and used the hyperparameters seen in our train.py script to finetune the model. 

We merged weights and utilized llama.cpp to convert our weights to a gguf file.

# Usage
Load the Jetson main file and the server main file onto their respective devices. 

# Recommended specs
Jetson on the drone, use a raspberry pi in a pinch. At least 8 GB of VRAM on the server GPU, ideally 16. 

# Debug
1: Check documentation
2: Try debug
3: Cry

# TODO
Write checks so the drone can't pitch and roll past 45-60 degrees