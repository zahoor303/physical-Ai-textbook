import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import ChatWidget from '@site/src/components/ChatWidget';
import AskSelectionButton from '@site/src/components/AskSelectionButton';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <BrowserOnly fallback={<div />}>
        {() => (
          <>
            <ChatWidget />
            <AskSelectionButton />
          </>
        )}
      </BrowserOnly>
    </>
  );
}
