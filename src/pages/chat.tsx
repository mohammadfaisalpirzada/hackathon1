import React, { useState } from "react";
import Layout from "@theme/Layout";

interface Answer {
  answer: string;
  sources?: Array<{ page: string; score: number }>;
}

export default function ChatPage() {
  // Echo chat (simple)
  const [msg, setMsg] = useState("");
  const [answer, setAnswer] = useState<string>("");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");

  // Selected-text chat (constrained)
  const [selectedText, setSelectedText] = useState("");
  const [question, setQuestion] = useState("");
  const [selAnswer, setSelAnswer] = useState<string>("");
  const [selLoading, setSelLoading] = useState(false);
  const [selError, setSelError] = useState<string>("");

  const API_BASE = "http://127.0.0.1:8000";

  async function send(): Promise<void> {
    setLoading(true);
    setError("");
    setAnswer("");

    try {
      const res = await fetch(`${API_BASE}/pdf_chat`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ message: msg }),
      });

      if (!res.ok) throw new Error(`HTTP ${res.status}`);

      const data = (await res.json()) as Answer;
      setAnswer(data.answer ?? "(no answer field)");
    } catch (e: unknown) {
      const errorMsg = e instanceof Error ? e.message : "Request failed";
      setError(errorMsg);
    } finally {
      setLoading(false);
    }
  }

  async function askOnText(): Promise<void> {
    setSelLoading(true);
    setSelError("");
    setSelAnswer("");

    try {
      const res = await fetch(`${API_BASE}/ask_on_text`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ selected_text: selectedText, question }),
      });

      if (!res.ok) throw new Error(`HTTP ${res.status}`);

      const data = (await res.json()) as Answer;
      setSelAnswer(data.answer ?? "(no answer field)");
    } catch (e: unknown) {
      const errorMsg = e instanceof Error ? e.message : "Request failed";
      setSelError(errorMsg);
    } finally {
      setSelLoading(false);
    }
  }

  const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>, callback: () => void): void => {
    if (e.ctrlKey && e.key === "Enter") {
      callback();
    }
  };

  return (
    <Layout title="Chatbot" description="Course chatbot">
      <main style={{ maxWidth: 900, margin: "0 auto", padding: "2rem 1rem" }}>
        <h1>🤖 Course Chatbot (Local)</h1>

        <p>
          Backend: <code>{API_BASE}</code>
        </p>

        <h2>Quick Chat</h2>
        <textarea
          value={msg}
          onChange={(e) => setMsg(e.target.value)}
          onKeyPress={(e) => handleKeyPress(e, send)}
          placeholder="Ask something... (Ctrl+Enter to send)"
          rows={4}
          style={{ width: "100%", padding: 12, fontFamily: "monospace" }}
        />

        <div style={{ marginTop: 12 }}>
          <button onClick={send} disabled={loading || !msg.trim()}>
            {loading ? "Sending..." : "Send"}
          </button>
        </div>

        {error ? (
          <p style={{ marginTop: 12, color: "#d9534f" }}>
            <b>❌ Error:</b> {error}
          </p>
        ) : null}

        {answer ? (
          <div style={{ marginTop: 16, backgroundColor: "#f5f5f5", padding: 16, borderRadius: 4 }}>
            <h3>✅ Answer</h3>
            <pre style={{ whiteSpace: "pre-wrap", fontFamily: "monospace", overflow: "auto" }}>
              {answer}
            </pre>
          </div>
        ) : null}

        <hr style={{ margin: "2rem 0" }} />

        <h2>Ask from Selected Text</h2>

        <label>
          <b>Selected Text</b>
        </label>
        <textarea
          value={selectedText}
          onChange={(e) => setSelectedText(e.target.value)}
          placeholder="Paste the selected paragraph here..."
          rows={8}
          style={{ width: "100%", padding: 12, fontFamily: "monospace" }}
        />

        <div style={{ marginTop: 12 }}>
          <label>
            <b>Question</b>
          </label>
          <input
            value={question}
            onChange={(e) => setQuestion(e.target.value)}
            onKeyPress={(e) => {
              if (e.key === "Enter") askOnText();
            }}
            placeholder='e.g., "summarize" or "What does this paragraph say?"'
            style={{ width: "100%", padding: 12, fontFamily: "monospace" }}
          />
        </div>

        <div style={{ marginTop: 12 }}>
          <button
            onClick={askOnText}
            disabled={selLoading || !selectedText.trim() || !question.trim()}
          >
            {selLoading ? "Asking..." : "Ask"}
          </button>
        </div>

        {selError ? (
          <p style={{ marginTop: 12, color: "#d9534f" }}>
            <b>❌ Error:</b> {selError}
          </p>
        ) : null}

        {selAnswer ? (
          <div style={{ marginTop: 16, backgroundColor: "#f5f5f5", padding: 16, borderRadius: 4 }}>
            <h3>✅ Answer (from selected text only)</h3>
            <pre style={{ whiteSpace: "pre-wrap", fontFamily: "monospace", overflow: "auto" }}>
              {selAnswer}
            </pre>
          </div>
        ) : null}
      </main>
    </Layout>
  );
}