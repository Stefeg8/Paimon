import os
import torch
import json
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig, TrainingArguments
from peft import LoraConfig, prepare_model_for_kbit_training, get_peft_model
from datasets import load_dataset, Dataset
from trl import SFTTrainer
import wandb
import warnings
import time

#TODO:
#Someone needs to scrape https://genshin-impact.fandom.com/wiki/Lore for literally everything it has
#Then add it to a json file and we can prepare it for training

# conversion commands after training
# First merge using pt_merge_weights.py
# use q8_0 for 4 bit precision instead of fp16
# Use 16 bit precision for more important shit and 4 bit for stuff that doesn't require as much accuracy
# change directory to llama.cpp first before executing
# https://github.com/ggerganov/llama.cpp <- model conversion directory
# python convert_hf_to_gguf.py ^
#      --outfile mistral_7b_genshin_v1.gguf ^
#      --outtype f16 ^
#      --model C:/Users/Steven/Desktop/Code/ECE5_Project/genshin_lore_training/merged_model
# Execute the following command when you're in the correct directory
# python convert_hf_to_gguf.py --outfile mistral_7b_genshin_v1.gguf --outtype f16 C:/Users/Steven/Desktop/Code/ECE5_Project/genshin_lore_training/merged_model

# For suppressing the torch autocast warning. Remove this when debugging
warnings.filterwarnings("ignore", category=FutureWarning, module="torch.*")

# Set up API keys (replace these with your actual API keys or set them via environment variables)
# You can also load these from a secret manager if available
WANDB_KEY = os.getenv("WANDB_KEY")

# Log in to Hugging Face and Weights & Biases
#!huggingface-cli login --token $HUGGINGFACE_TOKEN
# 402cfd8da726c5f814c84b212c6cdd87a1666830
wandb.login(key=WANDB_KEY)
run = wandb.init(
    project='Fine-tuning Mistral 7B - Genshin Lore',
    job_type="training",
    anonymous="allow"
)

# Define paths and settings
base_model = "models"
dataset_name = "path to dataset" # Dataset path
new_model = "mistral_7b_genshin_v1"

def preprocess_json_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        data = [json.loads(line) for line in f]
    
    processed_data = []
    for item in data:
        # Combine the fields into a single text for training
        combined_text = (
            # We can't do anything without any data lol
        )
        processed_data.append({"text": combined_text})
    
    return processed_data

# Example function to process character data
def preprocess_genshin_data(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    processed_data = []
    for item in data:
        prompt = f"Who is {item['character']}?"
        response = item['lore']
        processed_data.append({"prompt": prompt, "response": response})
    
    return processed_data

dataset_path = "dataset path"  # Dataset path
processed_data = preprocess_json_file(dataset_path)
# Load dataset
dataset = Dataset.from_dict({"text": [item["text"] for item in processed_data]}, split = "train")
# Load dataset
#dataset = load_dataset(dataset_name, split="train")

# Load model with 4-bit precision
bnb_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.float16,
    bnb_4bit_use_double_quant=False,
)
model = AutoModelForCausalLM.from_pretrained(
    base_model,
    #load_in_4bit=True,
    torch_dtype=torch.float16,
    device_map="auto",
    trust_remote_code=True, 
    quantization_config=bnb_config
)
model.config.use_cache = False
model.config.pretraining_tp = 1
model.gradient_checkpointing_enable()

# Load tokenizer
tokenizer = AutoTokenizer.from_pretrained(base_model, trust_remote_code=True)
tokenizer.padding_side = 'right'
tokenizer.pad_token = tokenizer.eos_token
tokenizer.add_eos_token = True

# Prepare model for k-bit training
model = prepare_model_for_kbit_training(model)
peft_config = LoraConfig(
    lora_alpha=16,
    lora_dropout=0.1,
    r=64,
    bias="none",
    task_type="CAUSAL_LM",
    target_modules=["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj"]
)
model = get_peft_model(model, peft_config)

# Define training arguments
# fp16 training allows for larger batch sizes
# TODO: Check vram usage with different batch sizes and adjust
training_args = TrainingArguments(
    output_dir="./results",
    num_train_epochs=1,
    per_device_train_batch_size=4,
    gradient_accumulation_steps=1,
    gradient_checkpointing=True,
    gradient_checkpointing_kwargs={'use_reentrant':False},
    optim="paged_adamw_32bit",
    save_steps=100,
    logging_steps=25,
    learning_rate=2e-4,
    weight_decay=0.001,
    fp16=True,
    bf16=False,
    max_grad_norm=0.3,
    max_steps=-1,
    warmup_ratio=0.03,
    group_by_length=True,
    lr_scheduler_type="constant",
    report_to="wandb"
)

# Initialize SFT Trainer
trainer = SFTTrainer(
    model=model,
    train_dataset=dataset,
    peft_config=peft_config,
    max_seq_length=None,
    dataset_text_field="text",
    tokenizer=tokenizer,
    args=training_args,
    packing=False,
)

checkpoint_dir = "./results"
resume_from_checkpoint = None

if os.path.exists(checkpoint_dir):
    checkpoints = [os.path.join(checkpoint_dir, cp) for cp in os.listdir(checkpoint_dir) if 'checkpoint' in cp]
    if checkpoints:
        resume_from_checkpoint = max(checkpoints, key=os.path.getctime)  # Get the latest checkpoint

# Train the model
#trainer.train()
try:
    time1 = time.time()
    trainer.train(resume_from_checkpoint=resume_from_checkpoint)
except KeyboardInterrupt:
    print("Training interrupted. Saving model...")
    time2 = time.time()
    print(f"Time taken: {time2-time1}")
    trainer.model.save_pretrained(new_model)

# Save the fine-tuned model
trainer.model.save_pretrained(new_model)
wandb.finish()
model.config.use_cache = True
