import React from 'react'
import ReactDOM from 'react-dom/client'
import Router from './Router.tsx'
import './index.css'

// Initialize theme from localStorage or default to dark
const savedTheme = localStorage.getItem('cubedynamics-theme');
const initialTheme = savedTheme ? JSON.parse(savedTheme).state?.theme : 'dark';
document.documentElement.setAttribute('data-theme', initialTheme);

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <Router />
  </React.StrictMode>,
)
