import React from 'react';

// Root component that wraps the entire app
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
    </>
  );
}