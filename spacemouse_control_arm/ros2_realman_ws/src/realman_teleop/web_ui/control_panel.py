"""
数采控制面板 Web UI。

启动后访问 http://localhost:5000 即可使用。
支持动态相机配置、日志显示、流畅的相机预览。
"""
import json
import time
import threading
import base64
from collections import deque
import numpy as np
from flask import Flask, jsonify, Response
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from cv_bridge import CvBridge
import cv2


# 内嵌 HTML 模板
INDEX_HTML = '''<!DOCTYPE html>
<html lang="zh">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>数采控制面板</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #1a1a2e; color: #eee; min-height: 100vh; padding: 15px; }
        .container { max-width: 1600px; margin: 0 auto; }
        h1 { text-align: center; margin-bottom: 10px; font-size: 20px; }
        .mode-badge { text-align: center; margin-bottom: 10px; }
        .badge { display: inline-block; padding: 4px 12px; border-radius: 12px; font-size: 12px; font-weight: bold; }
        .badge.dry-run { background: #fbbf24; color: #1a1a2e; }
        .badge.real { background: #4ade80; color: #1a1a2e; }
        .main-grid { display: grid; grid-template-columns: 280px 1fr 320px; gap: 12px; }
        @media (max-width: 1200px) { .main-grid { grid-template-columns: 1fr; } }
        .card { background: #16213e; border-radius: 10px; padding: 12px; }
        .card h2 { font-size: 11px; color: #888; margin-bottom: 10px; text-transform: uppercase; letter-spacing: 1px; }
        .btn { width: 100%; padding: 10px; border: none; border-radius: 6px; font-size: 13px; cursor: pointer; margin-bottom: 6px; transition: all 0.2s; }
        .btn:hover:not(:disabled) { transform: translateY(-1px); }
        .btn:disabled { opacity: 0.4; cursor: not-allowed; transform: none; }
        .btn-primary { background: #e94560; color: white; }
        .btn-success { background: #0f3460; color: white; }
        .btn-danger { background: #533483; color: white; }
        .btn-hint { font-size: 9px; color: #666; text-align: center; margin-top: -2px; margin-bottom: 6px; }
        .row { display: flex; gap: 6px; }
        .row .btn { flex: 1; }
        .node-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }
        .node-item { background: #0f3460; border-radius: 6px; padding: 6px; text-align: center; }
        .node-name { font-size: 9px; color: #888; margin-bottom: 2px; }
        .node-hz { font-size: 14px; font-weight: bold; }
        .node-hz span { font-size: 8px; font-weight: normal; color: #888; }
        .node-status { font-size: 8px; margin-top: 2px; }
        .node-status.active { color: #4ade80; }
        .node-status.inactive { color: #f87171; }
        .recorder-status { background: #0f3460; border-radius: 6px; padding: 10px; text-align: center; margin-bottom: 8px; }
        .recorder-state { font-size: 16px; font-weight: bold; margin-bottom: 4px; }
        .recorder-state.initializing { color: #fbbf24; }
        .recorder-state.waiting { color: #60a5fa; }
        .recorder-state.recording { color: #f87171; }
        .recorder-state.saving { color: #fbbf24; }
        .recorder-state.ready { color: #4ade80; }
        .recorder-state.offline { color: #6b7280; }
        .recorder-state.error { color: #ef4444; }
        .recorder-state.confirming { color: #fbbf24; }
        .modal-overlay { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.7); z-index: 1000; justify-content: center; align-items: center; }
        .modal-overlay.show { display: flex; }
        .modal-box { background: #16213e; border-radius: 12px; padding: 20px; max-width: 400px; text-align: center; }
        .modal-box h3 { margin-bottom: 10px; font-size: 16px; }
        .modal-box p { color: #888; font-size: 12px; margin-bottom: 15px; }
        .modal-box .row { justify-content: center; }
        .recorder-info { font-size: 10px; color: #888; }
        .camera-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 6px; }
        .camera-item { background: #0f3460; border-radius: 6px; overflow: hidden; }
        .camera-item img { width: 100%; height: auto; display: block; }
        .camera-label { font-size: 9px; color: #888; padding: 3px; text-align: center; background: rgba(0,0,0,0.3); }
        .log-panel { height: 100%; display: flex; flex-direction: column; }
        .log-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
        .log-header h2 { margin-bottom: 0; }
        .log-clear { background: #0f3460; border: none; color: #888; padding: 4px 8px; border-radius: 4px; cursor: pointer; font-size: 10px; }
        .log-clear:hover { color: #eee; }
        .log-container { flex: 1; background: #0a0a1a; border-radius: 6px; overflow-y: auto; max-height: 500px; font-family: monospace; font-size: 11px; }
        .log-entry { padding: 4px 8px; border-bottom: 1px solid #1a1a2e; word-break: break-all; }
        .log-entry.debug { color: #888; }
        .log-entry.info { color: #60a5fa; }
        .log-entry.warn { color: #fbbf24; }
        .log-entry.error { color: #f87171; background: rgba(248,113,113,0.1); }
        .log-entry.fatal { color: #ef4444; background: rgba(239,68,68,0.2); }
        .log-time { color: #666; margin-right: 6px; }
        .log-node { color: #888; margin-right: 6px; }
        .status-bar { text-align: center; padding: 6px; background: #0f3460; border-radius: 6px; font-size: 11px; margin-top: 12px; }
        .url-hint { text-align: center; font-size: 10px; color: #666; margin-top: 8px; }
        .vr-status { background: #0f3460; border-radius: 6px; padding: 10px; text-align: center; margin-bottom: 8px; }
        .vr-conn { font-size: 14px; font-weight: bold; margin-bottom: 4px; }
        .vr-conn.connected { color: #4ade80; }
        .vr-conn.disconnected { color: #f87171; }
        .vr-info { font-size: 10px; color: #888; }
        .vr-controls { margin-bottom: 8px; }
        .vr-bar { display: flex; align-items: center; gap: 6px; margin-bottom: 4px; font-size: 10px; }
        .vr-bar span:first-child { width: 50px; color: #888; }
        .bar-outer { flex: 1; height: 8px; background: #0f3460; border-radius: 4px; overflow: hidden; }
        .bar-inner { height: 100%; border-radius: 4px; transition: width 0.1s; }
        .bar-inner.trigger { background: linear-gradient(90deg, #e94560, #ff6b6b); }
        .bar-inner.grip { background: linear-gradient(90deg, #4ade80, #22c55e); }
        .bar-val { width: 35px; text-align: right; font-family: monospace; }
        .vr-pose { font-size: 9px; color: #666; text-align: center; font-family: monospace; }
        .arm-item { background: #0f3460; border-radius: 6px; padding: 8px; margin-bottom: 8px; }
        .arm-title { display: flex; justify-content: space-between; align-items: center; font-size: 12px; margin-bottom: 6px; }
        .arm-state { font-size: 10px; color: #888; }
        .arm-state.on { color: #4ade80; }
        .arm-state.off { color: #f87171; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 数采控制面板</h1>
        <div class="mode-badge">
            <span class="badge dry-run" id="mode-badge">仿真模式</span>
        </div>
        <div class="main-grid">
            <!-- 左列：控制 -->
            <div>
                <div class="card">
                    <h2>📊 节点状态</h2>
                    <div class="node-grid" id="node-grid"></div>
                </div>
                <div class="card" style="margin-top: 10px;">
                    <h2>📹 录制状态</h2>
                    <div class="recorder-status">
                        <div class="recorder-state offline" id="recorder-state">离线</div>
                        <div class="recorder-info" id="recorder-info">--</div>
                    </div>
                    <div class="row">
                        <button class="btn btn-success" id="rec-start-btn" onclick="callApi('/api/recorder/start', this)">▶️ 开始</button>
                        <button class="btn btn-danger" id="rec-stop-btn" onclick="showStopConfirm()">⏹️ 停止</button>
                    </div>
                </div>
                <div class="card" style="margin-top: 10px;">
                    <h2>🏠 机械臂 Home</h2>
                    <button class="btn btn-primary" id="home-btn" onclick="callHomeApi(this)">🏠 回到 Home 位姿</button>
                    <div class="btn-hint" id="home-hint">点击后机械臂移动到预设 home 位置</div>
                </div>
                <div class="card exo-card" id="exo-control-card" style="margin-top: 10px; display: none;">
                    <h2>🎮 机械臂控制（外骨骼模式）</h2>
                    <button class="btn btn-primary" id="enable-follow-btn" onclick="callApi('/api/enable_follow', this)" disabled>🟢 启用跟随</button>
                    <div class="btn-hint" id="enable-follow-hint">仿真模式无需启用</div>
                    <button class="btn btn-danger" id="disable-follow-btn" onclick="callApi('/api/disable_follow', this)" disabled>🔴 停止跟随</button>
                    <div class="btn-hint" id="disable-follow-hint">录制中无法停止</div>
                    <button class="btn btn-success" id="high-follow-btn" onclick="callApi('/api/high_follow', this)" disabled>⚡ 启用高跟随模式</button>
                    <div class="btn-hint" id="high-follow-hint">配置未启用软启动</div>
                </div>
                <div class="card" id="arm-control-card" style="margin-top: 10px;">
                    <h2>🦾 双臂控制</h2>
                    <div id="arm-list"></div>
                </div>
                <div class="card vr-card" id="vr-control-card" style="margin-top: 10px; display: none;">
                    <h2>🥽 VR 手柄状态</h2>
                    <div class="vr-status">
                        <div class="vr-conn" id="vr-conn">🔴 未连接</div>
                        <div class="vr-info" id="vr-info">--</div>
                    </div>
                    <div class="row" style="margin-bottom: 8px;">
                        <button class="btn btn-success" id="vr-enable-btn" onclick="callApi('/api/vr_enable', this)" disabled>🟢 启用VR控制</button>
                        <button class="btn btn-danger" id="vr-disable-btn" onclick="callApi('/api/vr_disable', this)" disabled>🔴 禁用VR控制</button>
                    </div>
                    <div class="btn-hint" id="vr-control-hint">等待 VR 连接...</div>
                    <div class="vr-controls">
                        <div class="vr-bar">
                            <span>Trigger</span>
                            <div class="bar-outer"><div class="bar-inner trigger" id="vr-trigger-bar"></div></div>
                            <span class="bar-val" id="vr-trigger-val">0.00</span>
                        </div>
                        <div class="vr-bar">
                            <span>Grip</span>
                            <div class="bar-outer"><div class="bar-inner grip" id="vr-grip-bar"></div></div>
                            <span class="bar-val" id="vr-grip-val">0.00</span>
                        </div>
                    </div>
                    <div class="vr-pose" id="vr-pose">位置: --</div>
                </div>
            </div>
            <!-- 中列：相机预览 -->
            <div class="card">
                <h2>📷 相机预览 (RGB + Depth)</h2>
                <div class="camera-grid" id="camera-grid"></div>
            </div>
            <!-- 右列：日志 -->
            <div class="card log-panel">
                <div class="log-header">
                    <h2>📋 系统日志</h2>
                    <button class="log-clear" onclick="clearLogs()">清空</button>
                </div>
                <div class="log-container" id="log-container"></div>
            </div>
        </div>
        <div class="status-bar" id="status">就绪</div>
        <div class="url-hint">🌐 http://localhost:5000</div>
        <!-- 停止确认对话框 -->
        <div class="modal-overlay" id="stop-confirm-modal">
            <div class="modal-box">
                <h3>📹 录制已停止</h3>
                <p id="stop-confirm-info">Episode 0 | 0 帧</p>
                <div class="row">
                    <button class="btn btn-success" onclick="confirmSave()">✅ 保存</button>
                    <button class="btn btn-danger" onclick="confirmDiscard()">🗑️ 丢弃</button>
                </div>
            </div>
        </div>
    </div>
    <script>
        const STATUS_MAP = {
            'initializing': '⏳ 初始化中...',
            'waiting': '⏸️ 等待开始',
            'recording': '🔴 录制中',
            'saving': '💾 保存中...',
            'confirming': '❓ 等待确认',
            'ready': '✅ 就绪',
            'offline': '⚫ 离线',
            'error': '❌ 初始化失败'
        };
        const LOG_LEVELS = ['debug', 'debug', 'info', 'warn', 'error', 'fatal'];
        let cameraElements = {};  // 缓存相机 img 元素
        let lastLogId = 0;
        let pendingStopInfo = null;  // 暂存停止时的录制信息
        let currentMode = 'exo';  // 当前模式：'exo' 或 'vr'
        let lastArmHtml = '';  // 避免高频刷新时重建按钮导致点击反馈被冲掉
        
        // 初始化：根据后端返回的模式显示对应的控制卡片
        function initializeMode(mode) {
            currentMode = mode;
            const exoCard = document.getElementById('exo-control-card');
            const vrCard = document.getElementById('vr-control-card');
            
            if (mode === 'vr') {
                // VR模式：显示VR卡片，隐藏外骨骼卡片
                if (exoCard) exoCard.style.display = 'none';
                if (vrCard) vrCard.style.display = 'block';
            } else {
                // 外骨骼模式：显示外骨骼卡片，隐藏VR卡片
                if (exoCard) exoCard.style.display = 'block';
                if (vrCard) vrCard.style.display = 'none';
            }
        }
        
        function showStopConfirm() {
            // 先暂停录制（进入 confirming 状态）
            fetch('/api/recorder/pause', { method: 'POST' }).then(() => {
                document.getElementById('stop-confirm-modal').classList.add('show');
            });
        }
        
        function confirmSave() {
            document.getElementById('stop-confirm-modal').classList.remove('show');
            fetch('/api/recorder/stop', { method: 'POST' }).then(res => res.json()).then(data => {
                document.getElementById('status').textContent = data.message;
            });
        }
        
        function confirmDiscard() {
            document.getElementById('stop-confirm-modal').classList.remove('show');
            fetch('/api/recorder/discard', { method: 'POST' }).then(res => res.json()).then(data => {
                document.getElementById('status').textContent = data.message;
            });
        }
        
        async function callApi(url, btn) {
            const status = document.getElementById('status');
            btn.disabled = true;
            status.textContent = '请求中...';
            try {
                const res = await fetch(url, { method: 'POST' });
                const data = await res.json();
                status.textContent = data.message;
            } catch (e) {
                status.textContent = '请求失败: ' + e.message;
            }
            setTimeout(() => { btn.disabled = false; }, 500);
        }

        async function callArmApi(arm, action, btn) {
            const status = document.getElementById('status');
            btn.disabled = true;
            status.textContent = '请求中...';
            try {
                const res = await fetch(`/api/arm/${arm}/${action}`, { method: 'POST' });
                const data = await res.json();
                status.textContent = data.message;
            } catch (e) {
                status.textContent = '请求失败: ' + e.message;
            }
            setTimeout(() => { btn.disabled = false; }, 500);
        }

        async function callHomeApi(btn) {
            const status = document.getElementById('status');
            const hint = document.getElementById('home-hint');
            btn.disabled = true;
            status.textContent = '正在移动到 home 位姿...';
            hint.textContent = '⏳ 机械臂移动中，请稍候...';
            try {
                const res = await fetch('/api/move_to_home', { method: 'POST' });
                const data = await res.json();
                status.textContent = data.message;
                if (data.success) {
                    hint.textContent = '✅ 已到达 home 位姿';
                } else {
                    hint.textContent = '❌ ' + data.message;
                }
            } catch (e) {
                status.textContent = '请求失败: ' + e.message;
                hint.textContent = '❌ 请求失败';
            }
            setTimeout(() => {
                btn.disabled = false;
                hint.textContent = '点击后机械臂移动到预设 home 位置';
            }, 2000);
        }
        
        function clearLogs() {
            document.getElementById('log-container').innerHTML = '';
        }
        
        async function updateStatus() {
            try {
                const res = await fetch('/api/status');
                const data = await res.json();
                
                // 初始化模式（仅第一次）
                if (typeof data.mode !== 'undefined' && currentMode !== data.mode) {
                    initializeMode(data.mode);
                }
                
                // 更新运行模式
                const modeBadge = document.getElementById('mode-badge');
                const drv = data.driver;
                const rec = data.recorder;  // 提前获取 rec
                if (drv.dry_run) {
                    modeBadge.textContent = '🔧 仿真模式';
                    modeBadge.className = 'badge dry-run';
                } else {
                    modeBadge.textContent = '🤖 真机模式';
                    modeBadge.className = 'badge real';
                }
                
                // 更新启用跟随按钮状态（真机模式安全开关）
                const efBtn = document.getElementById('enable-follow-btn');
                const efHint = document.getElementById('enable-follow-hint');
                const dfBtn = document.getElementById('disable-follow-btn');
                const dfHint = document.getElementById('disable-follow-hint');
                const isRecording = rec.status === 'recording' || rec.status === 'confirming';
                
                if (drv.dry_run) {
                    efBtn.disabled = true;
                    efHint.textContent = '仿真模式无需启用';
                    dfBtn.disabled = true;
                    dfHint.textContent = '仿真模式无需停止';
                } else if (drv.follow_enabled) {
                    efBtn.disabled = true;
                    efHint.textContent = '✅ 跟随已启用';
                    // 停止跟随按钮：录制中禁用
                    dfBtn.disabled = isRecording;
                    dfHint.textContent = isRecording ? '⚠️ 录制中无法停止跟随' : '点击停止跟随，可休息';
                } else {
                    efBtn.disabled = false;
                    efHint.textContent = '⚠️ 点击后机械臂开始跟随外骨骼';
                    dfBtn.disabled = true;
                    dfHint.textContent = '跟随未启用';
                }
                
                // 更新高跟随按钮状态
                const hfBtn = document.getElementById('high-follow-btn');
                const hfHint = document.getElementById('high-follow-hint');
                if (!drv.high_follow_enabled) {
                    hfBtn.disabled = true;
                    hfHint.textContent = '配置未启用软启动 (high_follow=false)';
                } else if (drv.high_follow_active) {
                    hfBtn.disabled = true;
                    hfHint.textContent = '✅ 高跟随已激活';
                } else {
                    hfBtn.disabled = false;
                    hfHint.textContent = '点击切换到高跟随模式';
                }
                
                // 更新节点状态
                const nodeGrid = document.getElementById('node-grid');
                let nodeHtml = '';
                for (const [name, info] of Object.entries(data.nodes)) {
                    const statusClass = info.active ? 'active' : 'inactive';
                    const statusText = info.active ? '在线' : '离线';
                    nodeHtml += `<div class="node-item">
                        <div class="node-name">${name}</div>
                        <div class="node-hz">${info.hz.toFixed(1)}<span> Hz</span></div>
                        <div class="node-status ${statusClass}">${statusText}</div>
                    </div>`;
                }
                nodeGrid.innerHTML = nodeHtml;
                
                // 更新录制状态
                const recorderState = document.getElementById('recorder-state');
                const recorderInfo = document.getElementById('recorder-info');
                recorderState.textContent = STATUS_MAP[rec.status] || rec.status;
                recorderState.className = 'recorder-state ' + rec.status;
                if (rec.status === 'recording' || rec.status === 'confirming') {
                    const dur = rec.duration || 0;
                    const min = Math.floor(dur / 60);
                    const sec = Math.floor(dur % 60);
                    const timeStr = min > 0 ? `${min}分${sec}秒` : `${sec}秒`;
                    recorderInfo.textContent = `Episode ${rec.episode} | ${rec.frame_count} 帧 | ${timeStr}`;
                    // 更新确认对话框信息
                    if (rec.status === 'confirming') {
                        document.getElementById('stop-confirm-info').textContent = `Episode ${rec.episode} | ${rec.frame_count} 帧 | ${timeStr}`;
                    }
                } else if (rec.status === 'saving') {
                    const savingEp = rec.saving_episode || '?';
                    const queueSize = rec.save_queue_size || 0;
                    if (queueSize > 0) {
                        recorderInfo.textContent = `正在存包 Episode ${savingEp}（队列: ${queueSize}）`;
                    } else {
                        recorderInfo.textContent = `正在存包 Episode ${savingEp}`;
                    }
                } else if (rec.status === 'waiting') {
                    recorderInfo.textContent = `等待录制 Episode ${rec.episode}`;
                } else if (rec.status === 'error') {
                    recorderInfo.textContent = rec.error_message || '初始化失败，请检查日志';
                } else {
                    recorderInfo.textContent = '--';
                }
                
                // 更新双臂控制面板
                const armList = document.getElementById('arm-list');
                const armNames = data.arm_names || Object.keys(data.arms || {});
                if (armList && armNames.length) {
                    let armHtml = '';
                    const vrDataForArms = data.vr || {};
                    const vrArms = vrDataForArms.arms || {};
                    const vrConnectedForArms = !!vrDataForArms.connected;
                    for (const arm of armNames) {
                        const armState = (data.arms && data.arms[arm]) ? data.arms[arm] : {};
                        const vrState = vrArms[arm] || {};
                        const followOn = !!armState.follow_enabled;
                        const vrOn = !!vrState.control_enabled;
                        const vrEnableDisabled = (!vrConnectedForArms || vrOn) ? 'disabled' : '';
                        const vrDisableDisabled = (!vrConnectedForArms || !vrOn) ? 'disabled' : '';
                        armHtml += `<div class="arm-item">
                            <div class="arm-title"><strong>${arm}</strong><span class="arm-state ${followOn || vrOn ? 'on' : 'off'}">${followOn ? '跟随中' : '未跟随'} · ${vrOn ? 'VR开' : 'VR关'}</span></div>
                            <div class="row">
                                <button class="btn btn-primary" onclick="callArmApi('${arm}', 'home', this)">Home</button>
                                <button class="btn btn-success" onclick="callArmApi('${arm}', 'enable_follow', this)">跟随</button>
                                <button class="btn btn-danger" onclick="callArmApi('${arm}', 'disable_follow', this)">停止</button>
                            </div>
                            <div class="row">
                                <button class="btn btn-success" ${vrEnableDisabled} onclick="callArmApi('${arm}', 'vr_enable', this)">VR启用</button>
                                <button class="btn btn-danger" ${vrDisableDisabled} onclick="callArmApi('${arm}', 'vr_disable', this)">VR禁用</button>
                            </div>
                        </div>`;
                    }
                    if (armHtml !== lastArmHtml) {
                        armList.innerHTML = armHtml;
                        lastArmHtml = armHtml;
                    }
                }

                // 更新录制按钮状态
                const startBtn = document.getElementById('rec-start-btn');
                const stopBtn = document.getElementById('rec-stop-btn');
                startBtn.disabled = rec.status !== 'waiting';
                stopBtn.disabled = rec.status !== 'recording';
                
                // 更新相机预览（只更新 src，不重建 DOM）
                const cameraGrid = document.getElementById('camera-grid');
                const cameraNames = Object.keys(data.cameras).sort();
                
                // 检查是否需要重建 DOM（相机数量变化）
                if (Object.keys(cameraElements).sort().join(',') !== cameraNames.join(',')) {
                    cameraElements = {};
                    let camHtml = '';
                    for (const name of cameraNames) {
                        camHtml += `<div class="camera-item">
                            <img id="cam-${name}" src="" alt="${name}">
                            <div class="camera-label">${name}</div>
                        </div>`;
                    }
                    if (!camHtml) {
                        camHtml = '<div class="camera-label" style="grid-column: 1/-1; padding: 20px;">无相机数据</div>';
                    }
                    cameraGrid.innerHTML = camHtml;
                    for (const name of cameraNames) {
                        cameraElements[name] = document.getElementById('cam-' + name);
                    }
                }
                
                // 只更新 src 属性
                for (const [name, imgData] of Object.entries(data.cameras)) {
                    if (cameraElements[name] && imgData) {
                        cameraElements[name].src = 'data:image/jpeg;base64,' + imgData;
                    }
                }
                
                // 更新日志
                if (data.logs && data.logs.length > 0) {
                    const logContainer = document.getElementById('log-container');
                    for (const log of data.logs) {
                        if (log.id > lastLogId) {
                            lastLogId = log.id;
                            const levelClass = LOG_LEVELS[log.level] || 'info';
                            const entry = document.createElement('div');
                            entry.className = 'log-entry ' + levelClass;
                            entry.innerHTML = `<span class="log-time">${log.time}</span><span class="log-node">[${log.node}]</span>${log.msg}`;
                            logContainer.appendChild(entry);
                        }
                    }
                    // 保持最多 200 条日志
                    while (logContainer.children.length > 200) {
                        logContainer.removeChild(logContainer.firstChild);
                    }
                    // 自动滚动到底部
                    logContainer.scrollTop = logContainer.scrollHeight;
                }
                
                // 更新 VR 状态
                const vr = data.vr;
                const vrConn = document.getElementById('vr-conn');
                const vrInfo = document.getElementById('vr-info');
                if (vr.connected) {
                    vrConn.textContent = '🟢 已连接';
                    vrConn.className = 'vr-conn connected';
                    const sourceMap = {'adb': 'USB/ADB', 'udp_binary': 'WiFi', 'udp_json': 'JSON'};
                    const srcName = sourceMap[vr.source] || vr.source;
                    vrInfo.textContent = `${srcName} | ${vr.device_id || '--'} | ${vr.recv_count} 条`;
                } else {
                    vrConn.textContent = '🔴 未连接';
                    vrConn.className = 'vr-conn disconnected';
                    vrInfo.textContent = '等待 VR 设备...';
                }
                // 更新 VR 控制按钮状态
                const vrEnableBtn = document.getElementById('vr-enable-btn');
                const vrDisableBtn = document.getElementById('vr-disable-btn');
                const vrControlHint = document.getElementById('vr-control-hint');
                if (!vr.connected) {
                    vrEnableBtn.disabled = true;
                    vrDisableBtn.disabled = true;
                    vrControlHint.textContent = '等待 VR 连接...';
                } else if (vr.control_enabled) {
                    vrEnableBtn.disabled = true;
                    vrDisableBtn.disabled = false;
                    vrControlHint.textContent = '✅ VR 控制已启用，按住 trigger 控制机械臂';
                } else {
                    vrEnableBtn.disabled = false;
                    vrDisableBtn.disabled = true;
                    vrControlHint.textContent = '点击启用 VR 控制';
                }
                // 更新进度条
                const triggerPct = Math.min(100, Math.max(0, vr.trigger * 100));
                const gripPct = Math.min(100, Math.max(0, vr.grip * 100));
                document.getElementById('vr-trigger-bar').style.width = triggerPct + '%';
                document.getElementById('vr-grip-bar').style.width = gripPct + '%';
                document.getElementById('vr-trigger-val').textContent = vr.trigger.toFixed(2);
                document.getElementById('vr-grip-val').textContent = vr.grip.toFixed(2);
                // 更新位姿
                const pos = vr.position;
                document.getElementById('vr-pose').textContent = `位置: (${pos[0]}, ${pos[1]}, ${pos[2]})`;
            } catch (e) {
                console.error('状态更新失败:', e);
            }
        }
        // WebUI 状态刷新；相机帧由后端限频，按钮 DOM 只在状态变化时更新
        setInterval(updateStatus, 100);
        updateStatus();
    </script>
</body>
</html>'''


class TopicMonitor:
    """话题监控器：统计消息频率。"""
    def __init__(self, window_sec: float = 2.0):
        self.window_sec = window_sec
        self._timestamps = []
        self._lock = threading.Lock()
    
    def on_msg(self, msg=None):
        now = time.time()
        with self._lock:
            self._timestamps.append(now)
            cutoff = now - self.window_sec
            self._timestamps = [t for t in self._timestamps if t > cutoff]
    
    def get_hz(self) -> float:
        now = time.time()
        with self._lock:
            cutoff = now - self.window_sec
            valid = [t for t in self._timestamps if t > cutoff]
            return len(valid) / self.window_sec if len(valid) >= 2 else 0.0
    
    def is_active(self, timeout: float = 1.0) -> bool:
        with self._lock:
            if not self._timestamps:
                return False
            return (time.time() - self._timestamps[-1]) < timeout


class ControlPanelNode(Node):
    """ROS2 节点：监控话题状态，调用 Service，收集日志。"""
    
    def __init__(self):
        super().__init__("control_panel_node")
        self.bridge = CvBridge()
        
        # 从参数读取运行模式
        self.declare_parameter("mode", "exo")  # "exo" 或 "vr"
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(f"🎮 运行模式: {self.mode}")
        
        # 从参数读取相机列表
        self.declare_parameter("camera_names", ["cam0", "cam1"])
        self.camera_names = self.get_parameter("camera_names").get_parameter_value().string_array_value
        if not self.camera_names:
            self.camera_names = ["cam0", "cam1"]
        self.get_logger().info(f"📷 相机列表: {self.camera_names}")

        # 是否显示 depth 画面
        self.declare_parameter("use_depth", False)
        self.use_depth = self.get_parameter("use_depth").get_parameter_value().bool_value
        
        self.declare_parameter("arm_names", ["arm"])
        self.declare_parameter("driver_service_prefixes", ["/driver"])
        self.declare_parameter("driver_status_topics", ["/driver/status"])
        self.declare_parameter("driver_state_topics", ["/rm/state_joint_state"])
        self.declare_parameter("vr_service_prefixes", ["/vr"])
        self.declare_parameter("vr_status_topics", ["/vr/teleop_status"])
        self.arm_names = list(self.get_parameter("arm_names").value) or ["arm"]
        self.driver_service_prefixes = list(self.get_parameter("driver_service_prefixes").value) or ["/driver"]
        self.driver_status_topics = list(self.get_parameter("driver_status_topics").value) or ["/driver/status"]
        self.driver_state_topics = list(self.get_parameter("driver_state_topics").value) or ["/rm/state_joint_state"]
        self.vr_service_prefixes = list(self.get_parameter("vr_service_prefixes").value) or ["/vr"]
        self.vr_status_topics = list(self.get_parameter("vr_status_topics").value) or ["/vr/teleop_status"]
        if len(self.driver_service_prefixes) == 1 and len(self.arm_names) > 1:
            self.driver_service_prefixes *= len(self.arm_names)
        if len(self.driver_status_topics) == 1 and len(self.arm_names) > 1:
            self.driver_status_topics *= len(self.arm_names)
        if len(self.driver_state_topics) == 1 and len(self.arm_names) > 1:
            self.driver_state_topics *= len(self.arm_names)
        if len(self.vr_service_prefixes) == 1 and len(self.arm_names) > 1:
            self.vr_service_prefixes *= len(self.arm_names)
        if len(self.vr_status_topics) == 1 and len(self.arm_names) > 1:
            self.vr_status_topics *= len(self.arm_names)

        # Service 客户端
        self.high_follow_clients = {name: self.create_client(Trigger, f"{prefix.rstrip('/')}/enable_high_follow") for name, prefix in zip(self.arm_names, self.driver_service_prefixes)}
        self.enable_follow_clients = {name: self.create_client(Trigger, f"{prefix.rstrip('/')}/enable_follow") for name, prefix in zip(self.arm_names, self.driver_service_prefixes)}
        self.disable_follow_clients = {name: self.create_client(Trigger, f"{prefix.rstrip('/')}/disable_follow") for name, prefix in zip(self.arm_names, self.driver_service_prefixes)}
        self.move_to_home_clients = {name: self.create_client(Trigger, f"{prefix.rstrip('/')}/move_to_home") for name, prefix in zip(self.arm_names, self.driver_service_prefixes)}
        self.high_follow_client = next(iter(self.high_follow_clients.values()))
        self.enable_follow_client = next(iter(self.enable_follow_clients.values()))
        self.disable_follow_client = next(iter(self.disable_follow_clients.values()))
        self.recorder_start_client = self.create_client(Trigger, "/recorder/start")
        self.recorder_stop_client = self.create_client(Trigger, "/recorder/stop")
        self.recorder_pause_client = self.create_client(Trigger, "/recorder/pause")
        self.recorder_discard_client = self.create_client(Trigger, "/recorder/discard")
        
        # VR 控制服务客户端
        self.vr_enable_control_clients = {name: self.create_client(Trigger, f"{prefix.rstrip('/')}/enable_control") for name, prefix in zip(self.arm_names, self.vr_service_prefixes)}
        self.vr_disable_control_clients = {name: self.create_client(Trigger, f"{prefix.rstrip('/')}/disable_control") for name, prefix in zip(self.arm_names, self.vr_service_prefixes)}
        self.vr_enable_control_client = next(iter(self.vr_enable_control_clients.values()))
        self.vr_disable_control_client = next(iter(self.vr_disable_control_clients.values()))

        # Home 服务客户端
        self.move_to_home_client = next(iter(self.move_to_home_clients.values()))
        
        # 节点监控器（根据模式配置）
        if self.mode == "vr":
            # VR模式：监控VR相关节点
            self.monitors = {
                "VR Input": TopicMonitor(),
                "VR Teleop": TopicMonitor(),
                **{f"Driver {name}": TopicMonitor() for name in self.arm_names},
            }
        else:
            # 外骨骼模式：监控外骨骼相关节点
            self.monitors = {
                "Encoder": TopicMonitor(),
                "Teleop": TopicMonitor(),
                **{f"Driver {name}": TopicMonitor() for name in self.arm_names},
            }
        
        # 添加相机监控
        for cam in self.camera_names:
            self.monitors[cam.capitalize()] = TopicMonitor()
        
        # Recorder 状态
        self._recorder_status = {"status": "offline", "episode": -1, "frame_count": 0, "recording": False, "duration": 0}
        self._recorder_last_update = 0.0
        self._recorder_lock = threading.Lock()
        
        # Driver 状态
        self._driver_status_by_arm = {name: {"dry_run": True, "follow_enabled": False, "high_follow_enabled": False, "high_follow_active": False} for name in self.arm_names}
        self._driver_status = self._driver_status_by_arm[self.arm_names[0]]
        self._driver_lock = threading.Lock()
        
        # VR 状态
        self._vr_status = {
            "connected": False,
            "source": "unknown",
            "device_id": "",
            "recv_count": 0,
            "trigger": 0.0,
            "grip": 0.0,
            "position": [0.0, 0.0, 0.0],
            "rotation": [0.0, 0.0, 0.0, 1.0],
            "control_enabled": False
        }
        self._vr_lock = threading.Lock()
        
        # 相机图像缓存
        self._camera_images = {}
        self._camera_lock = threading.Lock()
        self._last_cam_update = {}  # 限制更新频率
        
        # 日志缓存
        self._logs = deque(maxlen=100)
        self._log_lock = threading.Lock()
        self._log_id = 0
        
        # 订阅话题（根据模式）
        if self.mode == "vr":
            # VR模式：订阅VR相关话题
            self.create_subscription(PoseStamped, "/vr/right/pose",
                lambda m: self.monitors["VR Input"].on_msg(), 10)
            self.create_subscription(JointState, "/rm/action_joint_state",
                lambda m: self.monitors["VR Teleop"].on_msg(), 10)
            for name, state_topic in zip(self.arm_names, self.driver_state_topics):
                self.create_subscription(JointState, state_topic,
                    lambda m, n=name: self.monitors[f"Driver {n}"].on_msg(), 10)
        else:
            # 外骨骼模式：订阅外骨骼相关话题
            self.create_subscription(JointState, "/exo/joint_state", 
                lambda m: self.monitors["Encoder"].on_msg(), 10)
            self.create_subscription(JointState, "/rm/action_joint_state",
                lambda m: self.monitors["Teleop"].on_msg(), 10)
            for name, state_topic in zip(self.arm_names, self.driver_state_topics):
                self.create_subscription(JointState, state_topic,
                    lambda m, n=name: self.monitors[f"Driver {n}"].on_msg(), 10)
        
        # 动态订阅相机话题
        for cam in self.camera_names:
            self.create_subscription(
                Image, f"/camera/{cam}/color/image_raw",
                lambda m, c=cam: self._on_color(f"{c}_rgb", m), 10
            )
            if self.use_depth:
                self.create_subscription(
                    Image, f"/camera/{cam}/depth/image_raw",
                    lambda m, c=cam: self._on_depth(f"{c}_depth", m), 10
                )
        
        # Recorder 状态话题
        self.create_subscription(String, "/recorder/status", self._on_recorder_status, 10)
        
        # Driver 状态话题
        for name, topic in zip(self.arm_names, self.driver_status_topics):
            self.create_subscription(String, topic, lambda m, n=name: self._on_driver_status(m, n), 10)
        
        # VR 状态话题
        self.create_subscription(Bool, "/vr/connected", self._on_vr_connected, 10)
        self.create_subscription(String, "/vr/source_info", self._on_vr_source_info, 10)
        self.create_subscription(Float32, "/vr/right/trigger", self._on_vr_trigger, 10)
        self.create_subscription(Float32, "/vr/right/grip", self._on_vr_grip, 10)
        self.create_subscription(PoseStamped, "/vr/right/pose", self._on_vr_pose, 10)
        for name, topic in zip(self.arm_names, self.vr_status_topics):
            self.create_subscription(String, topic, lambda m, n=name: self._on_vr_teleop_status(m, n), 10)
        
        # 订阅 /rosout 获取日志
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Log, "/rosout", self._on_rosout, qos)
    
    def _on_color(self, name: str, msg: Image):
        """处理 RGB 图像，限制更新频率。"""
        cam_name = name.rsplit('_', 1)[0].capitalize()
        if cam_name in self.monitors:
            self.monitors[cam_name].on_msg()
        
        # 限制每个相机流的更新频率（约 30 FPS，与录制一致）
        now = time.time()
        if now - self._last_cam_update.get(name, 0) < 0.033:  # 33ms = 30Hz
            return
        self._last_cam_update[name] = now
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self._encode_and_store(name, cv_img)
        except Exception:
            pass
    
    def _on_depth(self, name: str, msg: Image):
        """处理深度图像，限制更新频率。"""
        now = time.time()
        if now - self._last_cam_update.get(name, 0) < 0.033:
            return
        self._last_cam_update[name] = now
        
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
            self._encode_and_store(name, cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB))
        except Exception:
            pass
    
    def _encode_and_store(self, name: str, cv_img):
        """缩放、编码并存储图像。"""
        h, w = cv_img.shape[:2]
        scale = min(320 / w, 240 / h)
        new_size = (int(w * scale), int(h * scale))
        cv_img = cv2.resize(cv_img, new_size, interpolation=cv2.INTER_NEAREST)
        _, buf = cv2.imencode('.jpg', cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 60])
        b64 = base64.b64encode(buf).decode('utf-8')
        with self._camera_lock:
            self._camera_images[name] = b64
    
    def _on_recorder_status(self, msg: String):
        """处理 Recorder 状态。"""
        try:
            data = json.loads(msg.data)
            with self._recorder_lock:
                self._recorder_status = data
                self._recorder_last_update = time.monotonic()
        except Exception:
            pass
    
    def _on_driver_status(self, msg: String, arm_name: str = None):
        """处理 Driver 状态。"""
        try:
            data = json.loads(msg.data)
            name = arm_name or self.arm_names[0]
            with self._driver_lock:
                self._driver_status_by_arm[name] = data
                self._driver_status = self._driver_status_by_arm[self.arm_names[0]]
        except Exception:
            pass
    
    def _on_vr_connected(self, msg: Bool):
        """处理 VR 连接状态。"""
        with self._vr_lock:
            self._vr_status["connected"] = msg.data
    
    def _on_vr_source_info(self, msg: String):
        """处理 VR 数据源信息。"""
        try:
            data = json.loads(msg.data)
            with self._vr_lock:
                self._vr_status["source"] = data.get("source", "unknown")
                self._vr_status["device_id"] = data.get("device_id", "")
                self._vr_status["recv_count"] = data.get("recv_count", 0)
        except Exception:
            pass
    
    def _on_vr_trigger(self, msg: Float32):
        """处理 VR Trigger 值。"""
        with self._vr_lock:
            self._vr_status["trigger"] = round(msg.data, 2)
    
    def _on_vr_grip(self, msg: Float32):
        """处理 VR Grip 值。"""
        with self._vr_lock:
            self._vr_status["grip"] = round(msg.data, 2)
    
    def _on_vr_pose(self, msg: PoseStamped):
        """处理 VR 位姿。"""
        with self._vr_lock:
            p = msg.pose.position
            q = msg.pose.orientation
            self._vr_status["position"] = [round(p.x, 3), round(p.y, 3), round(p.z, 3)]
            self._vr_status["rotation"] = [round(q.x, 3), round(q.y, 3), round(q.z, 3), round(q.w, 3)]
    
    def _on_vr_teleop_status(self, msg: String, arm_name: str = None):
        """处理 VR 遥操作状态（来自 vr_teleop_node）。"""
        try:
            data = json.loads(msg.data)
            name = arm_name or self.arm_names[0]
            with self._vr_lock:
                arms = self._vr_status.setdefault("arms", {})
                arms[name] = data
                self._vr_status["control_enabled"] = any(v.get("control_enabled", False) for v in arms.values())
        except Exception:
            pass
    
    def _on_rosout(self, msg: Log):
        """处理 /rosout 日志消息。"""
        # 过滤掉自己的日志和过于频繁的日志
        if msg.name == "control_panel_node":
            return
        
        # 只显示 INFO 及以上级别（level: 1=DEBUG, 2=INFO, 4=WARN, 8=ERROR, 16=FATAL）
        level_map = {1: 0, 2: 2, 4: 3, 8: 4, 16: 5}  # 映射到前端的 LOG_LEVELS 索引
        level = level_map.get(msg.level, 2)
        if level < 2:  # 跳过 DEBUG
            return
        
        # 格式化时间
        sec = msg.stamp.sec % 86400  # 只取当天秒数
        time_str = f"{sec // 3600:02d}:{(sec % 3600) // 60:02d}:{sec % 60:02d}"
        
        # 简化节点名
        node_name = msg.name.split('/')[-1] if '/' in msg.name else msg.name
        
        with self._log_lock:
            self._log_id += 1
            self._logs.append({
                "id": self._log_id,
                "time": time_str,
                "node": node_name,
                "level": level,
                "msg": msg.msg[:200]  # 限制消息长度
            })
    
    def call_service(self, client, timeout=3.0):
        """从 Flask 线程安全调用 Service。"""
        req = Trigger.Request()
        future = client.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return False, "调用超时"
            time.sleep(0.05)
        try:
            result = future.result()
            return (result.success, result.message) if result else (False, "无响应")
        except Exception as e:
            return False, f"调用失败: {e}"
    
    def get_full_status(self) -> dict:
        """获取完整状态。"""
        nodes = {name: {"active": mon.is_active(), "hz": round(mon.get_hz(), 1)} 
                 for name, mon in self.monitors.items()}
        with self._recorder_lock:
            recorder = self._recorder_status.copy()
            if time.monotonic() - self._recorder_last_update > 1.0:
                recorder = {
                    "status": "offline",
                    "episode": -1,
                    "frame_count": 0,
                    "recording": False,
                    "duration": 0,
                    "error_message": "\u5f55\u5236\u8282\u70b9\u65e0\u72b6\u6001\u6d88\u606f",
                }
        with self._driver_lock:
            driver = self._driver_status.copy()
            arm_status = {name: self._driver_status_by_arm.get(name, {}).copy() for name in self.arm_names}
        with self._camera_lock:
            cameras = self._camera_images.copy()
        with self._log_lock:
            logs = list(self._logs)
        with self._vr_lock:
            vr = self._vr_status.copy()
        return {
            "mode": self.mode,  # 添加模式信息
            "nodes": nodes, 
            "recorder": recorder, 
            "driver": driver, 
            "arms": arm_status,
            "arm_names": self.arm_names,
            "cameras": cameras, 
            "logs": logs, 
            "vr": vr
        }


# Flask 应用
app = Flask(__name__)
ros_node: ControlPanelNode = None


@app.route("/")
def index():
    return Response(INDEX_HTML, mimetype='text/html')


def _call_arm_client(arm_name: str, action: str):
    maps = {
        "home": ros_node.move_to_home_clients,
        "enable_follow": ros_node.enable_follow_clients,
        "disable_follow": ros_node.disable_follow_clients,
        "high_follow": ros_node.high_follow_clients,
        "vr_enable": ros_node.vr_enable_control_clients,
        "vr_disable": ros_node.vr_disable_control_clients,
    }
    clients = maps.get(action)
    if clients is None or arm_name not in clients:
        return False, f"未知机械臂或操作: {arm_name}/{action}"
    timeout = 30.0 if action == "home" else 3.0
    return ros_node.call_service(clients[arm_name], timeout=timeout)


@app.route("/api/arm/<arm_name>/<action>", methods=["POST"])
def arm_action(arm_name, action):
    success, msg = _call_arm_client(arm_name, action)
    return jsonify({"success": success, "message": f"[{arm_name}] {msg}"})


@app.route("/api/high_follow", methods=["POST"])
def enable_high_follow():
    success, msg = ros_node.call_service(ros_node.high_follow_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/enable_follow", methods=["POST"])
def enable_follow():
    success, msg = ros_node.call_service(ros_node.enable_follow_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/recorder/start", methods=["POST"])
def recorder_start():
    success, msg = ros_node.call_service(ros_node.recorder_start_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/recorder/stop", methods=["POST"])
def recorder_stop():
    success, msg = ros_node.call_service(ros_node.recorder_stop_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/recorder/pause", methods=["POST"])
def recorder_pause():
    success, msg = ros_node.call_service(ros_node.recorder_pause_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/recorder/discard", methods=["POST"])
def recorder_discard():
    success, msg = ros_node.call_service(ros_node.recorder_discard_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/disable_follow", methods=["POST"])
def disable_follow():
    success, msg = ros_node.call_service(ros_node.disable_follow_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/vr_enable", methods=["POST"])
def vr_enable():
    success, msg = ros_node.call_service(ros_node.vr_enable_control_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/vr_disable", methods=["POST"])
def vr_disable():
    success, msg = ros_node.call_service(ros_node.vr_disable_control_client)
    return jsonify({"success": success, "message": msg})


@app.route("/api/move_to_home", methods=["POST"])
def move_to_home():
    # 回 home 是阻塞操作，设置较长超时
    success, msg = ros_node.call_service(ros_node.move_to_home_client, timeout=30.0)
    return jsonify({"success": success, "message": msg})


@app.route("/api/status")
def get_status():
    return jsonify(ros_node.get_full_status())


def main():
    global ros_node
    rclpy.init()
    ros_node = ControlPanelNode()
    
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True),
        daemon=True
    )
    flask_thread.start()
    
    ros_node.get_logger().info("🌐 Web UI 已启动: http://localhost:5000")
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
