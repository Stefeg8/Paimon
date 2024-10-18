import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import PeftModel, PeftConfig

# Define the base model (replace with the correct model path or Hugging Face model name)
base_model_name = "models"
lora_weights_path = "mistral_7b_arXiv"  # Path to your LoRA weights
output_model_dir = "./merged_model"  # Directory to save the merged model

# Load the base model
base_model = AutoModelForCausalLM.from_pretrained(base_model_name)

# Load the LoRA adapter config and weights
peft_config = PeftConfig.from_pretrained(lora_weights_path)
peft_model = PeftModel.from_pretrained(base_model, lora_weights_path)

# Merge LoRA weights with the base model
peft_model = peft_model.merge_and_unload()

# Save the merged model
peft_model.save_pretrained(output_model_dir)

# Also, save the tokenizer
tokenizer = AutoTokenizer.from_pretrained(base_model_name)
tokenizer.save_pretrained(output_model_dir)

print(f"Merged model saved to {output_model_dir}")
