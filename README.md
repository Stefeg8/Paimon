# ECE5_Project
For our team, we propose a project in which we develop a drone with voice, transcription, and image detection capabilities. Through the utilization of already established models (YOLOv10, Mistral 7B, llama 3.2 1b or 3b, OpenAI whisper, CoquiTTS) readily made available, we plan on making the drone follow a specific person, being able take voice commands, and to converse with the user through the use of a LLM. We intend to fine tune the LLM so that it can provide responses resembling that of a video game character and to also create a shell/costume for the drone so that it also resembles the character.

This repository provides the relevant code for our project.

# Specifications
We use a Mistral 7B finetune to power our drone's responses to the user. This is complemented by CoquiTTS xtts_v2 voice cloning, allowing our drone to have a voice that mimics that of Paimon's from Genshin Impact. Through OpenAI's whisper, our drone can understand human speech and take that as input for the Mistral model. 

Using YOLOv10, we achieve vision capabilities for our drone, allowing it to safely navigate its surroundings. 