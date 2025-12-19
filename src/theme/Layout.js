import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatWidget from '../components/FloatingChatWidget';

const Layout = (props) => {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatWidget />
    </>
  );
};

export default Layout;