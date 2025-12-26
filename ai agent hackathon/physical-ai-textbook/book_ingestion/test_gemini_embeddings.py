import os
import google.generativeai as genai

def main():
    api_key = 'AIzaSyA5jyZrJ0SSBflYFuuBjMA01mMxrBMs4mQ'
    if not api_key:
        raise RuntimeError("GEMINI_API_KEY is missing!")

    print("🔑 GEMINI_API_KEY loaded successfully.")

    genai.configure(api_key=api_key)

    print("🤖 Testing Gemini embeddings...")

    res = genai.embed_content(
        model="models/text-embedding-004",
        content="Humanoid robots need ROS 2 for middleware.",
        task_type="retrieval_document",
    )

    print("Embedding vector length:", len(res["embedding"]))
    print("First 5 values:", res["embedding"][:5])

if __name__ == "__main__":
    main()
