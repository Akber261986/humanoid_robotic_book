import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1/intro_physical_ai">
            Start Reading the Book - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Educational book on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.aboutSection}>
          <div className="container">
            <div className="row">
              <div className="col col--6">
                <h2>About This Book</h2>
                <p>
                  This comprehensive guide covers everything you need to know about Physical AI and Humanoid Robotics,
                  from foundational concepts to advanced implementation techniques. Physical AI represents a paradigm
                  shift in artificial intelligence, moving beyond traditional software-based systems to encompass AI
                  systems that interact with and operate within the physical world.
                </p>
              </div>
              <div className="col col--6">
                <h2>What You'll Learn</h2>
                <ul>
                  <li>Foundations of Physical AI and embodied intelligence</li>
                  <li>Simulation environments (ROS 2, Gazebo, Isaac Sim)</li>
                  <li>Vision-Language Assistant (VLA) integration</li>
                  <li>Real-world deployment strategies</li>
                </ul>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}