import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './FloatingChatWidget.css';

const FloatingChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isExpanded, setIsExpanded] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your Humanoid Robotics assistant. Ask me anything about the book content!", sender: 'bot', timestamp: new Date() }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const {siteConfig = {}} = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl || '/';

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    if (!isOpen) {
      setIsOpen(true);
      // Small delay to allow the chat container to render before expanding
      setTimeout(() => setIsExpanded(true), 10);
    } else {
      setIsExpanded(false);
      // Wait for collapse animation to complete before closing
      setTimeout(() => setIsOpen(false), 300);
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Use the Railway backend endpoint
      const apiEndpoint = `https://humanoidroboticbook-production.up.railway.app/api/query-book?query=${encodeURIComponent(inputValue)}`;

      console.log('Sending request to:', apiEndpoint); // Debug log

      const response = await fetch(apiEndpoint, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      console.log('Response status:', response.status); // Debug log

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      console.log('API response:', data); // Debug log

      let botResponse;
      if (data.error) {
        botResponse = {
          id: Date.now() + 1,
          text: `Error: ${data.error}. Please try rephrasing your question.`,
          sender: 'bot',
          timestamp: new Date()
        };
      } else if (data.response) {
        // New API response with LLM-generated answer
        botResponse = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'bot',
          timestamp: new Date(),
          sources: data.sources || []
        };
      } else if (data.results && data.results.length > 0) {
        // Fallback to old format if new API isn't available
        const responseText = `I found some relevant information:\n\n${data.results.slice(0, 2).map(r => r.context).join('\n\n')}`;
        botResponse = {
          id: Date.now() + 1,
          text: responseText,
          sender: 'bot',
          timestamp: new Date(),
          sources: data.results
        };
      } else {
        botResponse = {
          id: Date.now() + 1,
          text: "I couldn't find specific information about your query in the book content. Please try rephrasing your question or ask about humanoid robotics fundamentals.",
          sender: 'bot',
          timestamp: new Date()
        };
      }

      setMessages(prev => [...prev, botResponse]);
    } catch (error) {
      console.error('Error sending message:', error);
      // Check for specific error types
      if (error.message.includes('CORS') || error.message.includes('network')) {
        const errorMessage = {
          id: Date.now() + 1,
          text: `Network error: Unable to connect to the backend. The API might not be available or CORS might not be configured properly.`,
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, errorMessage]);
      } else if (error.message.includes('404') || error.message.includes('405')) {
        // If the /api/query-book endpoint doesn't exist, try the legacy endpoint
        try {
          const fallbackEndpoint = `https://humanoidroboticbook-production.up.railway.app/query-book?query=${encodeURIComponent(inputValue)}`;
          console.log('Trying fallback endpoint:', fallbackEndpoint);

          const fallbackResponse = await fetch(fallbackEndpoint, {
            method: 'GET',
            headers: {
              'Content-Type': 'application/json',
            }
          });

          if (fallbackResponse.ok) {
            const fallbackData = await fallbackResponse.json();
            let fallbackBotResponse;
            if (fallbackData.error) {
              fallbackBotResponse = {
                id: Date.now() + 1,
                text: `Error: ${fallbackData.error}. Please try rephrasing your question.`,
                sender: 'bot',
                timestamp: new Date()
              };
            } else if (fallbackData.results && fallbackData.results.length > 0) {
              const responseText = `I found some relevant information:\n\n${fallbackData.results.slice(0, 2).map(r => r.context).join('\n\n')}`;
              fallbackBotResponse = {
                id: Date.now() + 1,
                text: responseText,
                sender: 'bot',
                timestamp: new Date(),
                sources: fallbackData.results
              };
            } else {
              fallbackBotResponse = {
                id: Date.now() + 1,
                text: "I couldn't find specific information about your query in the book content. Please try rephrasing your question or ask about humanoid robotics fundamentals.",
                sender: 'bot',
                timestamp: new Date()
              };
            }
            setMessages(prev => [...prev, fallbackBotResponse]);
            return; // Exit after handling fallback
          }
        } catch (fallbackError) {
          console.error('Fallback also failed:', fallbackError);
        }

        // If fallback also fails, send the original error message
        const errorMessage = {
          id: Date.now() + 1,
          text: `Sorry, I encountered an error processing your request: ${error.message}. Please try again.`,
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, errorMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: `Sorry, I encountered an error processing your request: ${error.message}. Please try again.`,
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className="floating-chat-widget">
      {isOpen ? (
        <div className={clsx('chat-container', { 'chat-expanded': isExpanded })}>
          <div className="chat-header">
            <h3>🤖 Humanoid Robotics Assistant</h3>
            <button className="chat-close-btn" onClick={toggleChat}>
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={clsx('message', {
                  'user-message': message.sender === 'user',
                  'bot-message': message.sender === 'bot'
                })}
              >
                <div className="message-text">{message.text}</div>
                <div className="message-time">{formatTime(message.timestamp)}</div>

                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources">
                    <details className="sources-details">
                      <summary>Sources</summary>
                      {message.sources.slice(0, 2).map((source, idx) => (
                        <div key={idx} className="source-item">
                          <strong>From {source.file || 'document'}:</strong> {source.context?.substring(0, 100)}...
                        </div>
                      ))}
                    </details>
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className="message bot-message">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-container">
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about humanoid robotics..."
              className="chat-input"
              rows="1"
              disabled={isLoading}
            />
            <button
              onClick={handleSendMessage}
              className="send-button"
              disabled={isLoading || !inputValue.trim()}
            >
              {isLoading ? 'Sending...' : '➤'}
            </button>
          </div>
        </div>
      ) : null}

      <button
        className={clsx('chat-fab', { 'chat-open': isOpen })}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M8 12H16M12 8V16M21 12C21 14.288 20.248 16.3903 18.941 18.0298C17.634 19.6693 15.873 20.722 13.9407 20.995C12.0085 21.268 10.0735 20.738 8.51757 19.5178C6.96164 18.2975 5.91418 16.4932 5.581 14.493C5.24783 12.4928 5.6545 10.4598 6.717 8.825C7.7795 7.19017 9.398 6.098 11.2 5.76C13.002 5.422 14.847 5.872 16.36 7" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M21 3L12 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      </button>
    </div>
  );
};

export default FloatingChatWidget;