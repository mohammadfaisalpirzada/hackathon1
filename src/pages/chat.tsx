import React, { useMemo, useState } from "react";
import Layout from "@theme/Layout";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

type Hit = {
  page: string;
  score: number;
  text: string;
};

type RagResponse = {
  ok?: boolean;
  collection?: string;
  threshold?: number;
  hits?: Hit[];
  answer?: string;
  error?: string;
  llm_error?: string;
};

export default function ChatPage(): JSX.Element {
  // ✅ Docusaurus way to read build-time config
  const { siteConfig } = useDocusaurusContext();
  const API_BASE = useMemo(() => {
    const cfg = (siteConfig.customFields as any) || {};
    // Falls back to local if not provided at build time
    return (cfg.apiBaseUrl as string) || "http://127.0.0.1:8000";
  }, [siteConfig]);

  const HIGHLIGHT_MIN_LEN = 3;

  // PDF RAG chat
  const [msg, setMsg] = useState("");
  const [answer, setAnswer] = useState<string>("");
  const [hits, setHits] = useState<Hit[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");

  // Selected-text chat (no LLM)
  const [selectedText, setSelectedText] = useState("");
  const [question, setQuestion] = useState("");
  const [selAnswer, setSelAnswer] = useState<string>("");
  const [selLoading, setSelLoading] = useState(false);
  const [selError, setSelError] = useState<string>("");

  const escapeRegExp = (value: string): string => value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");

  const getHighlightTokens = (query: string): string[] => {
    const tokens = (query || "")
      .trim()
      .split(/\s+/)
      .map((t) => t.trim())
      .filter((t) => t.length >= HIGHLIGHT_MIN_LEN);

    const seen = new Set<string>();
    const unique: string[] = [];
    for (const token of tokens) {
      const key = token.toLowerCase();
      if (seen.has(key)) continue;
      seen.add(key);
      unique.push(token);
    }

    unique.sort((a, b) => b.length - a.length);
    return unique;
  };

  const renderHighlightedText = (text: string, query: string): React.ReactNode => {
    const tokens = getHighlightTokens(query);
    if (!tokens.length) return text;

    const pattern = new RegExp(`(${tokens.map(escapeRegExp).join("|")})`, "gi");
    const parts = text.split(pattern);

    return parts.map((part, idx) => {
      if (idx % 2 === 1) {
        return (
          <mark
            key={idx}
            style={{
              backgroundColor: "#ffeb3b",
              padding: "0 2px",
              borderRadius: 2,
            }}
          >
            {part}
          </mark>
        );
      }
      return <React.Fragment key={idx}>{part}</React.Fragment>;
    });
  };

  async function postJson<T>(path: string, payload: any): Promise<T> {
    const url = `${API_BASE}${path}`;
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });

    const text = await res.text(); // read once
    if (!res.ok) {
      throw new Error(`HTTP ${res.status} ${res.statusText}${text ? ` - ${text}` : ""}`);
    }

    try {
      return JSON.parse(text) as T;
    } catch {
      throw new Error(`Invalid JSON from server: ${text.slice(0, 400)}`);
    }
  }

  function extractRag(data: RagResponse): { answer: string; hits: Hit[]; warn: string } {
    const a = (data.answer || "").trim();
    const h = Array.isArray(data.hits) ? data.hits : [];
    const serverErr = (data.error || "").trim();
    const llmErr = (data.llm_error || "").trim();

    let warn = "";
    if (serverErr) warn = serverErr;
    else if (llmErr) warn = `LLM fallback used: ${llmErr}`;

    return { answer: a, hits: h, warn };
  }

  async function send(): Promise<void> {
    const q = msg.trim();
    if (!q) return;

    setLoading(true);
    setError("");
    setAnswer("");
    setHits([]);

    try {
      // ✅ Prefer /pdf_answer
      const data = await postJson<RagResponse>("/pdf_answer", { message: q });
      const out = extractRag(data);

      setAnswer(out.answer || "No answer returned.");
      setHits(out.hits);

      // If backend says Gemini failed (llm_error) we still show answer but warn
      if (out.warn) setError(out.warn);
    } catch (e1: unknown) {
      // ✅ Fallback to /pdf_chat if /pdf_answer totally fails
      try {
        const data2 = await postJson<RagResponse>("/pdf_chat", { message: q });
        const out2 = extractRag(data2);
        setAnswer(out2.answer || "No answer returned.");
        setHits(out2.hits);

        const errMsg = e1 instanceof Error ? e1.message : "Request failed";
        setError(`pdf_answer failed, used pdf_chat. Details: ${errMsg}`);
      } catch (e2: unknown) {
        const errMsg2 = e2 instanceof Error ? e2.message : "Request failed";
        setError(errMsg2);
      }
    } finally {
      setLoading(false);
    }
  }

  async function askOnText(): Promise<void> {
    const t = selectedText.trim();
    const q = question.trim();
    if (!t || !q) return;

    setSelLoading(true);
    setSelError("");
    setSelAnswer("");

    try {
      const data = await postJson<{ answer?: string; error?: string }>("/ask_on_text", {
        selected_text: t,
        question: q,
      });

      const a = (data.answer || "").trim();
      if (!a) {
        setSelError((data.error || "No answer returned.").trim());
      } else {
        setSelAnswer(a);
      }
    } catch (e: unknown) {
      const errorMsg = e instanceof Error ? e.message : "Request failed";
      setSelError(errorMsg);
    } finally {
      setSelLoading(false);
    }
  }

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>, callback: () => void): void => {
    if ((e as any).nativeEvent?.isComposing) return;
    if ((e.ctrlKey || e.metaKey) && e.key === "Enter") {
      e.preventDefault();
      callback();
    }
  };

  return (
    <Layout title="Chatbot" description="Course chatbot">
      <main style={{ maxWidth: 900, margin: "0 auto", padding: "2rem 1rem" }}>
        <h1>🤖 Course Chatbot</h1>

        <p>
          Backend: <code>{API_BASE}</code>
        </p>

        <h2>PDF Chat</h2>
        <textarea
          value={msg}
          onChange={(e) => setMsg(e.target.value)}
          onKeyDown={(e) => handleKeyDown(e, send)}
          placeholder="Ask something... (Ctrl+Enter to send)"
          rows={4}
          style={{ width: "100%", padding: 12, fontFamily: "monospace" }}
        />

        <div style={{ marginTop: 12, display: "flex", gap: 10, alignItems: "center" }}>
          <button onClick={send} disabled={loading || !msg.trim()}>
            {loading ? "Sending..." : "Send"}
          </button>

          <button
            onClick={() => {
              setMsg("");
              setAnswer("");
              setHits([]);
              setError("");
            }}
            disabled={loading}
          >
            Clear
          </button>
        </div>

        {error ? (
          <p style={{ marginTop: 12, color: "#d9534f" }}>
            <b>⚠️ Note:</b> {error}
          </p>
        ) : null}

        {answer ? (
          <div style={{ marginTop: 16, backgroundColor: "#f5f5f5", padding: 16, borderRadius: 6 }}>
            <h3>✅ Answer</h3>
            <pre style={{ whiteSpace: "pre-wrap", fontFamily: "monospace", overflow: "auto" }}>
              {renderHighlightedText(answer, msg)}
            </pre>

            {hits.length ? (
              <div style={{ marginTop: 14 }}>
                <b>Top matches:</b>
                <ul style={{ marginTop: 8 }}>
                  {hits.map((h, i) => (
                    <li key={i} style={{ marginBottom: 10 }}>
                      <div>
                        <b>Page {h.page}</b> (score {Number(h.score).toFixed(3)})
                      </div>
                      <div style={{ fontFamily: "monospace", whiteSpace: "pre-wrap" }}>
                        {renderHighlightedText(h.text, msg)}
                      </div>
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
            onKeyDown={(e) => {
              if ((e as any).nativeEvent?.isComposing) return;
              if (e.key === "Enter") {
                e.preventDefault();
                askOnText();
              }
            }}
            placeholder='e.g., "summarize" or "What does this paragraph say?"'
            style={{ width: "100%", padding: 12, fontFamily: "monospace" }}
          />
        </div>

        <div style={{ marginTop: 12, display: "flex", gap: 10, alignItems: "center" }}>
          <button onClick={askOnText} disabled={selLoading || !selectedText.trim() || !question.trim()}>
            {selLoading ? "Asking..." : "Ask"}
          </button>

          <button
            onClick={() => {
              setSelectedText("");
              setQuestion("");
              setSelAnswer("");
              setSelError("");
            }}
            disabled={selLoading}
          >
            Clear
          </button>
        </div>

        {selError ? (
          <p style={{ marginTop: 12, color: "#d9534f" }}>
            <b>❌ Error:</b> {selError}
          </p>
        ) : null}

        {selAnswer ? (
          <div style={{ marginTop: 16, backgroundColor: "#f5f5f5", padding: 16, borderRadius: 6 }}>
            <h3>✅ Answer (from selected text only)</h3>
            <pre style={{ whiteSpace: "pre-wrap", fontFamily: "monospace", overflow: "auto" }}>
              {renderHighlightedText(selAnswer, question)}
            </pre>
          </div>
        ) : null}
      </main>
    </Layout>
  );
}
