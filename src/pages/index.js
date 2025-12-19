import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1/intro_physical_ai">
            Start Reading - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="An Educational Book on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Physical AI Foundations</h3>
                <p>Learn the principles of Physical AI and embodied intelligence.</p>
              </div>
              <div className="col col--4">
                <h3>Humanoid Robotics</h3>
                <p>Understand the fundamentals and concepts of humanoid robotics.</p>
              </div>
              <div className="col col--4">
                <h3>Practical Applications</h3>
                <p>Explore real-world applications and implementation strategies.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}