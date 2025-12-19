import React, { useState } from 'react';
import clsx from 'clsx';
import './FloatingChatWidget.css';

const FloatingChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isExpanded, setIsExpanded] = useState(false);

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
          <div className="chat-content">
            <iframe
              src="/chat"
              title="Humanoid Robotics Chat"
              className="chat-iframe"
              frameBorder="0"
              allow="microphone"
            />
          </div>
        </div>
      ) : null}

      <button
        className={clsx('chat-fab', { 'chat-open': isOpen })}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M8 12H16M12 8V16M21 12C21 14.288 20.248 16.3903 18.941 18.0298C17.634 19.6693 15.873 20.722 13.9407 20.995C12.0085 21.268 10.0735 20.738 8.51757 19.5178C6.96164 18.2975 5.91418 16.4932 5.581 14.493C5.24783 12.4928 5.6545 10.4598 6.717 8.825C7.7795 7.19017 9.398 6.098 11.2 5.76C13.002 5.422 14.847 5.872 16.36 7" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M21 3L12 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        )}
      </button>
    </div>
  );
};

export default FloatingChatWidget;