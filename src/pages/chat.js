import React, { useState, useEffect, useRef } from 'react';
import Layout from '@theme/Layout';
import './chat.css';

const API_BASE = 'https://humanoidroboticbook-production.up.railway.app';

function ChatPage() {
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your Humanoid Robotics assistant. I specialize in answering questions about humanoid robotics, kinematics, control systems, and locomotion. Feel free to ask me anything about the book content!",
      sender: 'bot',
      timestamp: new Date()
    },
    {
      id: 2,
      text: "Tip: You can ask about specific topics like 'inverse kinematics', 'balance control', 'walking algorithms', or 'sensor fusion'. I'll find the most relevant information from the book for you.",
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const [status, setStatus] = useState('Ready to use');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const addMessage = (text, sender, sources = null) => {
    const newMessage = {
      id: Date.now(),
      text,
      sender,
      timestamp: new Date(),
      sources
    };
    setMessages(prev => [...prev, newMessage]);
  };

  const sendMessage = async () => {
    const message = inputValue.trim();
    if (!message || isLoading) return;

    addMessage(message, 'user');
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_BASE}/query-book?query=${encodeURIComponent(message)}`);
      const data = await response.json();

      if (data.error) {
        addMessage(`⚠️ I encountered an issue: ${data.error}`, 'bot');
      } else if (data.results && data.results.length > 0) {
        const responseText = `✅ I found some relevant information about "${data.query}":`;
        addMessage(responseText, 'bot', data.results);
      } else {
        addMessage(`🤔 I couldn't find specific information about "${message}" in the book content. Please try rephrasing your question or ask about humanoid robotics fundamentals like kinematics, locomotion, or control systems.`, 'bot');
      }
    } catch (error) {
      addMessage(`⚠️ Connection error: ${error.message}. Please check your internet connection and try again.`, 'bot');
      console.error('API Error:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const triggerIndex = async () => {
    if (isProcessing) return;

    setIsProcessing(true);
    setStatus('Processing book content...');

    try {
      const response = await fetch(`${API_BASE}/trigger-index`, { method: 'POST' });
      const data = await response.json();

      if (data.status === 'success') {
        setStatus('✅ Book content processed successfully!');
        addMessage('📚 Book content has been processed successfully!', 'bot');
      } else {
        setStatus(`❌ Error: ${data.stderr || data.message}`);
        addMessage(`⚠️ Error processing content: ${data.stderr || data.message}`, 'bot');
      }
    } catch (error) {
      setStatus(`❌ Error: ${error.message}`);
      addMessage(`⚠️ Error: ${error.message}`, 'bot');
    } finally {
      setIsProcessing(false);
    }
  };

  const embedToQdrant = async () => {
    if (isProcessing) return;

    setIsProcessing(true);
    setStatus('Embedding book content to Qdrant...');

    try {
      const response = await fetch(`${API_BASE}/embed-book`, { method: 'POST' });
      const data = await response.json();

      if (data.status === 'success') {
        setStatus('✅ Book content embedded to Qdrant successfully!');
        addMessage('💾 Book content has been embedded to Qdrant successfully! Vector search is now available.', 'bot');
      } else {
        setStatus(`❌ Error: ${data.stderr || data.message}`);
        addMessage(`⚠️ Error embedding to Qdrant: ${data.stderr || data.message}`, 'bot');
      }
    } catch (error) {
      setStatus(`❌ Error: ${error.message}`);
      addMessage(`⚠️ Error: ${error.message}`, 'bot');
    } finally {
      setIsProcessing(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  const predefinedQuestions = [
    { text: "What are the key components of a humanoid robot?", icon: "🧠" },
    { text: "Explain bipedal locomotion principles.", icon: "🚶" },
    { text: "How does inverse kinematics work in robotics?", icon: "📐" },
    { text: "What are the main challenges in humanoid robotics?", icon: "⚠️" }
  ];

  const handlePredefinedQuestion = (question) => {
    setInputValue(question);
    setTimeout(() => {
      sendMessage();
    }, 100);
  };

  return (
    <Layout title="Ask the Book" description="Chat with the Humanoid Robotics Book">
      <div className="chat-page">
        <div className="container padding-horiz--md">
          <div className="chat-container">
            <div className="sidebar">
              <h3>🤖 Assistant Controls</h3>

              <div className="welcome-card">
                <h2>💡 Welcome!</h2>
                <p>I'm your Humanoid Robotics expert. Ask me about kinematics, control systems, locomotion, or any topic from the book.</p>

                <div className="quick-actions">
                  {predefinedQuestions.map((q, index) => (
                    <div
                      key={index}
                      className="quick-action-btn"
                      onClick={() => handlePredefinedQuestion(q.text)}
                    >
                      <span>{q.icon}</span> {q.text.length > 20 ? q.text.substring(0, 20) + '...' : q.text}
                    </div>
                  ))}
                </div>
              </div>

              <button
                className="btn btn-outline"
                onClick={triggerIndex}
                disabled={isProcessing}
              >
                📚 Process Book Content
              </button>
              <button
                className="btn"
                onClick={embedToQdrant}
                disabled={isProcessing}
              >
                🎯 Embed to Qdrant
              </button>

              <div className="status-section">
                <h3>📊 System Status</h3>
                <div className={`status ${status.includes('❌') ? 'status-error' : ''}`}>
                  <div className="status-item">
                    <span className="status-label">Status:</span>
                    <span className="status-value">{status.includes('❌') ? 'Error' : 'Ready'}</span>
                  </div>
                  <div className="status-item">
                    <span className="status-label">Connection:</span>
                    <span className="status-value">{status.includes('❌') ? 'Inactive' : 'Active'}</span>
                  </div>
                  <div className="status-item">
                    <span className="status-label">Model:</span>
                    <span className="status-value">Gemini 1.5 Flash</span>
                  </div>
                  <div className="status-item">
                    <span className="status-label">Last Update:</span>
                    <span className="status-value">{new Date().toLocaleTimeString()}</span>
                  </div>
                </div>
              </div>
            </div>

            <div className="chat-area">
              <div className="messages">
                {messages.map((message) => (
                  <div key={message.id} className={`message ${message.sender}-message`}>
                    <span className="message-icon">
                      {message.sender === 'user' ? '👤 ' : '🤖 '}
                    </span>
                    {message.text}
                    <div className="message-time">{formatTime(message.timestamp)}</div>
                    {message.sources && (
                      <div className="sources">
                        <div className="sources-title">📚 Sources:</div>
                        {message.sources.slice(0, 2).map((source, idx) => (
                          <div key={idx} className="source-item">
                            <strong>{source.file}:</strong> {source.context.substring(0, 150)}...
                          </div>
                        ))}
                      </div>
                    )}
                  </div>
                ))}
                {isLoading && (
                  <div className="message bot-message">
                    <span className="message-icon">🤖 </span>
                    <div className="typing-indicator">
                      <span></span>
                      <span></span>
                      <span></span> Thinking...
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>
              <div className="input-area">
                <div className="input-container">
                  <input
                    type="text"
                    className="chat-input"
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    onKeyPress={handleKeyPress}
                    placeholder="Ask about humanoid robotics, kinematics, control systems..."
                    disabled={isLoading}
                  />
                  <button
                    className="send-btn"
                    onClick={sendMessage}
                    disabled={isLoading || !inputValue.trim()}
                  >
                    ✈️ Send
                  </button>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ChatPage;