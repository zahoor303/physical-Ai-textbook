from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    gemini_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "physical_ai_book"

    model_embedding: str = "models/text-embedding-004"
    model_chat: str = "models/gemini-2.5-flash"

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


settings = Settings()