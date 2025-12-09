import React from 'react';
import FloatingChatWidget from '../components/FloatingChatWidget';

// Root component that wraps the entire app
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <FloatingChatWidget />
    </>
  );
}