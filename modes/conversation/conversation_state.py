import os
from openai import AzureOpenAI
from dotenv import load_dotenv

load_dotenv()

# Initialize the Azure OpenAI Client
azure_client = AzureOpenAI(
    azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"),
    api_key=os.getenv("AZURE_OPENAI_API_KEY"),
    api_version="2024-08-01-preview"
)
AZURE_DEPLOYMENT = os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME", "gpt-4o")

# Maintain conversation memory globally in this module
chat_history = [
    {
        "role": "system", 
        "content": "You are Dhruv, an AI companion robot built by Daksh Dynamics. Keep your answers fun, witty, and strictly under 2 sentences. You are having a casual conversation. Do not use any emojis in your response."
    }
]

def generate_chat_response(user_text: str, visual_context: str = "") -> str:
    """Takes user text, queries GPT-4o with memory, and returns the AI's response."""
    contextualized_text = user_text
    if visual_context:
        contextualized_text = f"{visual_context}\nUser says: {user_text}"
        
    chat_history.append({"role": "user", "content": contextualized_text})
    print("[Brain] Thinking...")
    
    try:
        response = azure_client.chat.completions.create(
            model=AZURE_DEPLOYMENT,
            messages=chat_history,
            temperature=0.7,
            max_tokens=150
        )
        
        ai_response = response.choices[0].message.content
        chat_history.append({"role": "assistant", "content": ai_response})
        return ai_response
        
    except Exception as e:
        print(f"[AI Chat] Error: {e}")
        return "Sorry, my brain is having a glitch."