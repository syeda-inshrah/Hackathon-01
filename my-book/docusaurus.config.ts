import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Hackathon Book', // FIX: Updated site title
  tagline: 'Documentation for our awesome project',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, 
  },

  // FIX: Set the production url and base URL for GitHub Pages deployment
  url: 'https://syeda-inshrah.github.io',
  baseUrl: '/Hackathon-01/',

  // FIX: GitHub pages deployment config.
  organizationName: 'syeda-inshrah', // Your GitHub user name
  projectName: 'Hackathon-01', // Your repository name
  deploymentBranch: 'gh-pages', // Docusaurus deploys to this branch

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          // FIX: Changed to .js to match the file we generated earlier
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/syeda-inshrah/Hackathon-01/tree/main/my-book/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/syeda-inshrah/Hackathon-01/tree/main/my-book/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Hackathon Book', // FIX: Updated navbar title
      logo: {
        alt: 'Hackathon Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book Sections', // Updated label for clarity
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/syeda-inshrah/Hackathon-01', // FIX: Updated GitHub link
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Book Introduction',
              // 🛑 FIX: Link to the new file: 01-introduction.md
              to: '/docs/01-introduction', 
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'X',
              href: 'https://x.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/syeda-inshrah/Hackathon-01', // FIX: Updated GitHub link
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} My Hackathon Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;