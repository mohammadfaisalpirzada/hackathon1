import React, { useState } from "react";
import Layout from "@theme/Layout";

interface Source {
  page: string;
  score: number;
}

interface Answer {
  answer: string;
  sources?: Source[];
}

export default function ChatPage() {
  const API_BASE = "http://127.0.0.1:8000";

  // PDF RAG chat
  const [msg, setMsg] = useState("");
  const [answer, setAnswer] = useState<string>("");
  const [sources, setSources] = useState<Source[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");

  // Selected-text chat (constrained)
  const [selectedText, setSelectedText] = useState("");
  const [question, setQuestion] = useState("");
  const [selAnswer, setSelAnswer] = useState<string>("");
  const [selLoading, setSelLoading] = useState(false);
  const [selError, setSelError] = useState<string>("");

  async function callApi(path: string, payload: any): Promise<Answer> {
    const res = await fetch(`${API_BASE}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });

    // try to read error body for better debugging
    if (!res.ok) {
      let bodyText = "";
      try {
        bodyText = await res.text();
      } catch {}
      throw new Error(`HTTP ${res.status} ${res.statusText}${bodyText ? ` - ${bodyText}` : ""}`);
    }

    return (await res.json()) as Answer;
  }

  async function send(): Promise<void> {
    setLoading(true);
    setError("");
    setAnswer("");
    setSources([]);

    const payload = { message: msg };

    try {
      // Try clean answer first
      const data = await callApi("/pdf_answer", payload);
      setAnswer(data.answer ?? "(no answer field)");
      setSources(data.sources ?? []);
    } catch (e1: unknown) {
      // If /pdf_answer fails (Gemini missing/500), fallback to snippets
      try {
        const data2 = await callApi("/pdf_chat", payload);
        setAnswer(data2.answer ?? "(no answer field)");
        setSources(data2.sources ?? []);
        const errMsg = e1 instanceof Error ? e1.message : "Request failed";
        setError(`pdf_answer failed, used pdf_chat fallback. Details: ${errMsg}`);
      } catch (e2: unknown) {
        const errMsg2 = e2 instanceof Error ? e2.message : "Request failed";
        setError(errMsg2);
      }
    } finally {
      setLoading(false);
    }
  }

  async function askOnText(): Promise<void> {
    setSelLoading(true);
    setSelError("");
    setSelAnswer("");

    try {
      const data = await callApi("/ask_on_text", {
        selected_text: selectedText,
        question,
      });
      setSelAnswer(data.answer ?? "(no answer field)");
    } catch (e: unknown) {
      const errorMsg = e instanceof Error ? e.message : "Request failed";
      setSelError(errorMsg);
    } finally {
      setSelLoading(false);
    }
  }

  const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>, callback: () => void): void => {
    if (e.ctrlKey && e.key === "Enter") callback();
  };

  return (
    <Layout title="Chatbot" description="Course chatbot">
      <main style={{ maxWidth: 900, margin: "0 auto", padding: "2rem 1rem" }}>
        <h1>🤖 Course Chatbot (Local)</h1>

        <p>
          Backend: <code>{API_BASE}</code>
        </p>

        <h2>PDF Chat</h2>
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

            {sources.length ? (
              <div style={{ marginTop: 12 }}>
                <b>Sources:</b>
                <ul style={{ marginTop: 8 }}>
                  {sources.map((s, i) => (
                    <li key={i}>
                      Page {s.page} (score {Number(s.score).toFixed(3)})
                    </li>
                  ))}
                </ul>
              </div>
            ) : null}
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
          <button onClick={askOnText} disabled={selLoading || !selectedText.trim() || !question.trim()}>
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
