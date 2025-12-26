import React, { useState, useEffect } from "react";

const API_BASE = "http://localhost:8000"; // replace in production

export default function AskSelectionButton() {
  const [showButton, setShowButton] = useState(false);
  const [coords, setCoords] = useState<{ x: number; y: number } | null>(null);
  const [selectedText, setSelectedText] = useState("");
  const [modalOpen, setModalOpen] = useState(false);
  const [question, setQuestion] = useState("");
  const [answer, setAnswer] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  // Detect user text selection
  useEffect(() => {
    const handleMouseUp = () => {
      const sel = window.getSelection();
      const text = sel?.toString().trim();

      if (text && sel?.rangeCount) {
        const range = sel.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(text);
        setCoords({ x: rect.left + rect.width / 2, y: rect.top - 10 });
        setShowButton(true);
      } else {
        setShowButton(false);
      }
    };

    document.addEventListener("mouseup", handleMouseUp);
    return () => document.removeEventListener("mouseup", handleMouseUp);
  }, []);

  function closeModal() {
    setModalOpen(false);
    setQuestion("");
    setAnswer(null);
  }

  async function askAI() {
    setLoading(true);
    setAnswer(null);

    const query = question || "Explain this text in simple terms.";

    try {
      const res = await fetch(`${API_BASE}/ask-on-selection`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          selection_text: selectedText,
          question: query,
        }),
      });

      const data = await res.json();
      setAnswer(data.answer);
    } catch (err) {
      setAnswer("Something went wrong.");
    } finally {
      setLoading(false);
    }
  }

  return (
    <>
      {showButton && coords && (
        <button
          onClick={() => setModalOpen(true)}
          style={{
            position: "fixed",
            top: coords.y,
            left: coords.x,
            transform: "translate(-50%, -100%)",
            padding: "6px 12px",
            background: "#2563EB",
            color: "white",
            borderRadius: "8px",
            border: "none",
            cursor: "pointer",
            zIndex: 9999,
            boxShadow: "0 4px 12px rgba(0,0,0,0.2)",
          }}
        >
          Ask AI
        </button>
      )}

      {modalOpen && (
        <div
          style={{
            position: "fixed",
            inset: 0,
            background: "rgba(0, 0, 0, 0.5)",
            backdropFilter: "blur(4px)",
            zIndex: 99999,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            padding: "20px",
          }}
          onClick={closeModal}
        >
          <div
            onClick={(e) => e.stopPropagation()}
            style={{
              background: "white",
              borderRadius: "16px",
              width: "100%",
              maxWidth: "600px",
              maxHeight: "85vh",
              display: "flex",
              flexDirection: "column",
              boxShadow: "0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04)",
              overflow: "hidden",
            }}
          >
            {/* Header */}
            <div
              style={{
                padding: "24px 24px 20px",
                borderBottom: "1px solid #e5e7eb",
                display: "flex",
                justifyContent: "space-between",
                alignItems: "center",
              }}
            >
              <h3
                style={{
                  margin: 0,
                  fontSize: "1.25rem",
                  fontWeight: 600,
                  color: "#111827",
                }}
              >
                Ask AI About This Text
              </h3>
              <button
                onClick={closeModal}
                style={{
                  background: "none",
                  border: "none",
                  fontSize: "1.5rem",
                  color: "#6b7280",
                  cursor: "pointer",
                  padding: "0",
                  width: "32px",
                  height: "32px",
                  borderRadius: "6px",
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  transition: "all 0.2s",
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.background = "#f3f4f6";
                  e.currentTarget.style.color = "#111827";
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.background = "none";
                  e.currentTarget.style.color = "#6b7280";
                }}
              >
                ×
              </button>
            </div>

            {/* Content */}
            <div
              style={{
                padding: "24px",
                overflowY: "auto",
                flex: 1,
              }}
            >
              {/* Selected Text */}
              <div style={{ marginBottom: "20px" }}>
                <label
                  style={{
                    display: "block",
                    fontSize: "0.875rem",
                    fontWeight: 500,
                    color: "#374151",
                    marginBottom: "8px",
                  }}
                >
                  Selected Text
                </label>
                <div
                  style={{
                    background: "#f9fafb",
                    border: "1px solid #e5e7eb",
                    padding: "16px",
                    borderRadius: "8px",
                    fontSize: "0.9375rem",
                    lineHeight: "1.6",
                    color: "#1f2937",
                    maxHeight: "150px",
                    overflowY: "auto",
                  }}
                >
                  {selectedText}
                </div>
              </div>

              {/* Question Input */}
              <div style={{ marginBottom: "20px" }}>
                <label
                  style={{
                    display: "block",
                    fontSize: "0.875rem",
                    fontWeight: 500,
                    color: "#374151",
                    marginBottom: "8px",
                  }}
                >
                  Your Question (optional)
                </label>
                <textarea
                  placeholder="e.g., Can you explain this in simpler terms?"
                  value={question}
                  onChange={(e) => setQuestion(e.target.value)}
                  rows={3}
                  style={{
                    width: "100%",
                    padding: "12px",
                    fontSize: "0.9375rem",
                    borderRadius: "8px",
                    border: "1px solid #d1d5db",
                    outline: "none",
                    transition: "border-color 0.2s, box-shadow 0.2s",
                    resize: "vertical",
                    fontFamily: "inherit",
                    lineHeight: "1.5",
                  }}
                  onFocus={(e) => {
                    e.currentTarget.style.borderColor = "#2563eb";
                    e.currentTarget.style.boxShadow = "0 0 0 3px rgba(37, 99, 235, 0.1)";
                  }}
                  onBlur={(e) => {
                    e.currentTarget.style.borderColor = "#d1d5db";
                    e.currentTarget.style.boxShadow = "none";
                  }}
                />
              </div>

              {/* Answer Section */}
              {answer && (
                <div style={{ marginTop: "24px" }}>
                  <label
                    style={{
                      display: "block",
                      fontSize: "0.875rem",
                      fontWeight: 500,
                      color: "#374151",
                      marginBottom: "8px",
                    }}
                  >
                    AI Response
                  </label>
                  <div
                    style={{
                      background: "#eff6ff",
                      border: "1px solid #bfdbfe",
                      padding: "16px",
                      borderRadius: "8px",
                      fontSize: "0.9375rem",
                      lineHeight: "1.7",
                      color: "#1e3a8a",
                      whiteSpace: "pre-wrap",
                    }}
                  >
                    {answer}
                  </div>
                </div>
              )}
            </div>

            {/* Footer */}
            <div
              style={{
                padding: "16px 24px",
                borderTop: "1px solid #e5e7eb",
                background: "#f9fafb",
                display: "flex",
                justifyContent: "flex-end",
                gap: "12px",
              }}
            >
              <button
                onClick={closeModal}
                style={{
                  padding: "10px 20px",
                  borderRadius: "8px",
                  border: "1px solid #d1d5db",
                  background: "white",
                  color: "#374151",
                  fontSize: "0.9375rem",
                  fontWeight: 500,
                  cursor: "pointer",
                  transition: "all 0.2s",
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.background = "#f9fafb";
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.background = "white";
                }}
              >
                Close
              </button>
              <button
                onClick={askAI}
                disabled={loading}
                style={{
                  padding: "10px 24px",
                  borderRadius: "8px",
                  border: "none",
                  background: loading ? "#9ca3af" : "#2563eb",
                  color: "white",
                  fontSize: "0.9375rem",
                  fontWeight: 500,
                  cursor: loading ? "not-allowed" : "pointer",
                  transition: "all 0.2s",
                  opacity: loading ? 0.7 : 1,
                }}
                onMouseEnter={(e) => {
                  if (!loading) e.currentTarget.style.background = "#1d4ed8";
                }}
                onMouseLeave={(e) => {
                  if (!loading) e.currentTarget.style.background = "#2563eb";
                }}
              >
                {loading ? "Thinking..." : "Get Answer"}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}