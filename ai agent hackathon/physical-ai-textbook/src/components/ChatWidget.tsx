import React, { useState } from "react";

type Message = { role: "user" | "assistant"; content: string };

const API_BASE = "http://localhost:8000";

export default function ChatWidget() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState(true);
  const [isExpanded, setIsExpanded] = useState(false);

  function clearChat() {
    setMessages([]);
    setError(null);
  }

  async function sendMessage(e: React.FormEvent) {
    e.preventDefault();
    if (!input.trim()) return;

    const userMsg: Message = { role: "user", content: input };
    setMessages((prev) => [...prev, userMsg]);
    setInput("");
    setError(null);
    setLoading(true);

    try {
      const res = await fetch(`${API_BASE}/chat`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          query: userMsg.content,
          top_k: 5,
        }),
      });

      if (!res.ok) {
        throw new Error(`HTTP ${res.status}`);
      }

      const data = await res.json();
      const assistantMsg: Message = {
        role: "assistant",
        content: data.answer,
      };
      setMessages((prev) => [...prev, assistantMsg]);
    } catch (err: any) {
      console.error(err);
      setError("Something went wrong talking to the AI.");
    } finally {
      setLoading(false);
    }
  }

  return (
    <>
      {!isOpen && (
        <button
          onClick={() => setIsOpen(true)}
          style={{
            position: "fixed",
            bottom: "1.5rem",
            right: "1.5rem",
            width: "60px",
            height: "60px",
            borderRadius: "50%",
            background: "#3B82F6",
            color: "white",
            border: "none",
            fontSize: "1.5rem",
            cursor: "pointer",
            boxShadow: "0 4px 12px rgba(0,0,0,0.3)",
            zIndex: 9999,
          }}
        >
          💬
        </button>
      )}

      {isOpen && isExpanded && (
        <div
          onClick={() => setIsExpanded(false)}
          style={{
            position: "fixed",
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            background: "rgba(0, 0, 0, 0.5)",
            zIndex: 9998,
          }}
        />
      )}

      {isOpen && (
        <div
          style={{
            position: "fixed",
            bottom: "1.5rem",
            right: "1.5rem",
            width: isExpanded ? "50vw" : "400px",
            maxHeight: isExpanded ? "85vh" : "400px",
            background: "#111827",
            color: "white",
            borderRadius: "0.75rem",
            boxShadow: "0 10px 30px rgba(0,0,0,0.5)",
            display: "flex",
            flexDirection: "column",
            overflow: "hidden",
            zIndex: 9999,
            fontSize: "0.9rem",
            transition: "width 0.3s ease, max-height 0.3s ease",
          }}
        >
          <div
            style={{
              padding: "0.75rem 1rem",
              borderBottom: "1px solid #374151",
              fontWeight: 600,
              display: "flex",
              alignItems: "center",
              justifyContent: "space-between",
            }}
          >
            <span>🤖 Physical AI Tutor</span>
            <div style={{ display: "flex", gap: "0.5rem" }}>
              <button
                onClick={() => setIsExpanded(!isExpanded)}
                title={isExpanded ? "Shrink" : "Expand"}
                style={{
                  background: "transparent",
                  border: "none",
                  color: "#9CA3AF",
                  cursor: "pointer",
                  fontSize: "1.2rem",
                  padding: "0.25rem",
                }}
              >
                {isExpanded ? "⤓" : "⤢"}
              </button>
              <button
                onClick={clearChat}
                title="Clear chat"
                style={{
                  background: "transparent",
                  border: "none",
                  color: "#9CA3AF",
                  cursor: "pointer",
                  fontSize: "1.2rem",
                  padding: "0.25rem",
                }}
              >
                🗑️
              </button>
              <button
                onClick={() => setIsOpen(false)}
                title="Close chat"
                style={{
                  background: "transparent",
                  border: "none",
                  color: "#9CA3AF",
                  cursor: "pointer",
                  fontSize: "1.2rem",
                  padding: "0.25rem",
                }}
              >
                ✕
              </button>
            </div>
          </div>

      <div
        style={{
          flex: 1,
          padding: "0.75rem",
          overflowY: "auto",
          gap: "0.5rem",
          display: "flex",
          flexDirection: "column",
          position: "relative",
        }}
      >
        {messages.length === 0 && (
          <div style={{ color: "#9CA3AF" }}>
            Ask anything about the Physical AI & Humanoid Robotics textbook.
          </div>
        )}

        {messages.map((m, i) => (
          <div
            key={i}
            style={{
              alignSelf: m.role === "user" ? "flex-end" : "flex-start",
              background: m.role === "user" ? "#4B5563" : "#1F2937",
              padding: "0.5rem 0.75rem",
              borderRadius: "0.75rem",
              maxWidth: "80%",
              whiteSpace: "pre-wrap",
            }}
          >
            {m.content}
          </div>
        ))}

        {loading && (
          <div
            style={{
              display: "flex",
              alignItems: "center",
              gap: "0.5rem",
              color: "#9CA3AF",
            }}
          >
            <div
              style={{
                width: "8px",
                height: "8px",
                borderRadius: "50%",
                background: "#3B82F6",
                animation: "pulse 1.5s ease-in-out infinite",
              }}
            />
            <style>
              {`
                @keyframes pulse {
                  0%, 100% { opacity: 0.3; transform: scale(0.8); }
                  50% { opacity: 1; transform: scale(1.2); }
                }
              `}
            </style>
            Thinking...
          </div>
        )}

        {error && (
          <div style={{ color: "#FCA5A5" }}>
            {error}
          </div>
        )}
      </div>

      <form
        onSubmit={sendMessage}
        style={{
          borderTop: "1px solid #374151",
          padding: "0.5rem",
          display: "flex",
          gap: "0.5rem",
        }}
      >
        <input
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask about the book…"
          style={{
            flex: 1,
            borderRadius: "999px",
            border: "none",
            padding: "0.4rem 0.75rem",
            fontSize: "0.9rem",
            outline: "none",
          }}
        />
        <button
          type="submit"
          disabled={loading}
          style={{
            borderRadius: "999px",
            border: "none",
            padding: "0.4rem 0.9rem",
            background: loading ? "#6B7280" : "#3B82F6",
            color: "white",
            fontWeight: 500,
            cursor: loading ? "default" : "pointer",
          }}
        >
          {loading ? "…" : "Send"}
        </button>
      </form>
        </div>
      )}
    </>
  );
}