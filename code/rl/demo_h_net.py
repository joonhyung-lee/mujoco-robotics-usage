"""
This code is a demo of the H-Net architecture for reinforcement learning.
    - Mamba-2 architecture for the main network
    - Dynamic chunking module for the input
    - Dechunking module for the output
    - PPO algorithm for training
    - MuJoCo physics engine for the environment
    - PyTorch library for the neural network
    - tqdm library for the progress bar

mjpython demo_h_net.py
"""
import os
import mujoco
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import random
from tqdm import trange
import traceback
import cv2  # For video saving

# --- MuJoCo 환경 설정 ---
MODEL_PATH = "./ant.xml"
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Ant model not found at {MODEL_PATH}. Download from https://github.com/deepmind/mujoco")

# MuJoCo 모델 및 데이터 초기화
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)
state_dim = 16  # 8 hinge joint angles (qpos[7:]) + 8 velocities (qvel[6:])
action_dim = model.nu  # 8 actuators
max_steps = 1000
timestep = model.opt.timestep

# 렌더링 설정
USE_VIEWER = True  # Set to True to use mujoco.viewer, False for mujoco.Renderer
renderer = None
viewer = None
video_writer = None

if USE_VIEWER:
    try:
        print("[main] Initializing mujoco.viewer")
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
        print("[main] mujoco.viewer initialized successfully")
    except Exception as e:
        print(f"[main] mujoco.viewer initialization failed: {e}")
        print(traceback.format_exc())
        print("[main] Running without viewer.")
else:
    try:
        print("[main] Initializing mujoco.Renderer")
        renderer = mujoco.Renderer(model)
        # Initialize video writer
        video_path = "./simulation.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_path, fourcc, 30.0, (640, 480))
        print(f"[main] Video will be saved to {video_path}")
    except Exception as e:
        print(f"[main] mujoco.Renderer initialization failed: {e}")
        print(traceback.format_exc())
        print("[main] Running without rendering.")

# --- 데이터 전처리 ---
def get_state(data):
    """현재 상태: 8 hinge joint angles (qpos[7:]) + 8 velocities (qvel[6:])"""
    return np.concatenate([data.qpos[7:], data.qvel[6:]])

def float_to_byte_array(state, action, bits=8):
    """실수 값을 8비트 바이트 배열로 변환"""
    state_low = np.array([-10] * state_dim)
    state_high = np.array([10] * state_dim)
    action_low = np.array([-1] * action_dim)
    action_high = np.array([1] * action_dim)
    state = np.clip((state - state_low) / (state_high - state_low), 0, 1)
    action = np.where(np.isnan(action), 0, action)  # Replace NaN with 0
    action = np.clip((action - action_low) / (action_high - action_low), 0, 1)
    state_bytes = np.round(state * (2**bits - 1)).astype(np.uint8).tobytes()
    action_bytes = np.round(action * (2**bits - 1)).astype(np.uint8).tobytes()
    return state_bytes + action_bytes

# --- H-Net 아키텍처 ---
class MambaLayer(nn.Module):
    """간단한 Mamba-2 레이어"""
    def __init__(self, d_model, d_state=16, expand=2):
        super(MambaLayer, self).__init__()
        self.d_model = d_model
        self.d_state = d_state
        self.expand = expand
        self.linear_xz = nn.Linear(d_model, 2 * expand * d_model)
        self.linear_bca = nn.Linear(d_model, 2 * d_state + d_model)
        self.conv = nn.Conv1d(2 * expand * d_model, 2 * expand * d_model, kernel_size=3, padding=1)
        self.out_proj = nn.Linear(2 * expand * d_model, d_model)

    def forward(self, x):
        batch, seq_len, _ = x.shape
        xz = self.linear_xz(x)
        bca = self.linear_bca(x)
        x = self.conv(xz.permute(0, 2, 1)).permute(0, 2, 1)
        out = self.out_proj(x)
        return out

class DynamicChunking(nn.Module):
    """동적 청킹 모듈: 고정된 수의 인덱스 선택"""
    def __init__(self, d_model, downsample_factor=3):
        super(DynamicChunking, self).__init__()
        self.routing = nn.Linear(d_model, 1)
        self.downsample_factor = downsample_factor
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        batch, seq_len, d_model = x.shape
        boundary_probs = self.sigmoid(self.routing(x))  # [batch, seq_len, 1]
        num_chunks = max(1, seq_len // self.downsample_factor)  # 고정된 청크 수

        downsampled = []
        for b in range(batch):
            probs = boundary_probs[b].squeeze(-1)  # [seq_len]
            _, indices = torch.topk(probs, k=num_chunks, dim=0)
            indices = indices.sort().values  # 정렬된 인덱스
            downsampled_seq = x[b][indices]  # [num_chunks, d_model]
            downsampled.append(downsampled_seq)
        
        downsampled = torch.stack(downsampled)  # [batch, num_chunks, d_model]
        return downsampled, boundary_probs

class Dechunking(nn.Module):
    """디청킹 모듈"""
    def __init__(self, d_model):
        super(Dechunking, self).__init__()
        self.smoothing = nn.Linear(d_model, d_model)
        self.upsample = nn.Linear(d_model, d_model)

    def forward(self, x, boundary_probs, original_len):
        smoothed = self.smoothing(x)
        upsampled = torch.zeros(x.shape[0], original_len, x.shape[-1], device=x.device)
        for b in range(x.shape[0]):
            indices = torch.linspace(0, x.shape[1]-1, original_len).long().to(x.device)
            upsampled[b] = smoothed[b, indices]
        return self.upsample(upsampled)

class HNet(nn.Module):
    """1-stage H-Net"""
    def __init__(self, d_model=512, downsample_factor=3):
        super(HNet, self).__init__()
        self.input_proj = nn.Linear(1, d_model)
        self.encoder = nn.Sequential(
            MambaLayer(d_model),
            nn.LayerNorm(d_model)
        )
        self.chunking = DynamicChunking(d_model, downsample_factor)
        self.main_network = nn.Sequential(
            MambaLayer(d_model),
            nn.TransformerEncoderLayer(d_model=d_model, nhead=8),
            nn.LayerNorm(d_model)
        )
        self.dechunking = Dechunking(d_model)
        self.output_layer = nn.Linear(d_model, action_dim)
        self.value_layer = nn.Linear(d_model, 1)  # Value function for PPO
        self.pool = nn.AdaptiveAvgPool1d(1)

    def forward(self, x, original_len):
        x = self.input_proj(x)  # [batch, seq_len, d_model]
        x = self.encoder(x)
        x, boundary_probs = self.chunking(x)
        x = self.main_network(x)
        x = self.dechunking(x, boundary_probs, original_len)
        action = self.output_layer(x)  # [batch, seq_len, action_dim]
        value = self.value_layer(x)  # [batch, seq_len, 1]
        action = self.pool(action.permute(0, 2, 1)).squeeze(-1)  # [batch, action_dim]
        value = self.pool(value.permute(0, 2, 1)).squeeze(-1)  # [batch, 1]
        action = torch.tanh(action)  # [batch, action_dim]
        return action, value, boundary_probs

# --- 강화 학습(PPO) 설정 ---
class PPO:
    def __init__(self, model, lr=1e-4, gamma=0.99, clip_eps=0.3):
        self.model = model
        self.optimizer = optim.AdamW(model.parameters(), lr=lr)
        self.gamma = gamma
        self.clip_eps = clip_eps
        self.memory = deque(maxlen=10000)
        self.last_loss = None  # 마지막 loss 저장

    def store(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def compute_gae(self, rewards, values, next_values, dones):
        advantages = []
        gae = 0
        for r, v, nv, d in zip(reversed(rewards), reversed(values), reversed(next_values), reversed(dones)):
            delta = r + self.gamma * nv * (1 - d) - v
            gae = delta + self.gamma * 0.95 * gae * (1 - d)
            advantages.insert(0, gae)
        return torch.tensor(advantages, dtype=torch.float32)

    def update(self, batch_size=64):
        if len(self.memory) < batch_size:
            self.last_loss = None
            return
        batch = random.sample(self.memory, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        states = torch.tensor(np.array([np.frombuffer(s, dtype=np.uint8) for s in states]), dtype=torch.float32).reshape(batch_size, -1, 1)
        actions = torch.tensor(np.array(actions), dtype=torch.float32)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        next_states = torch.tensor(np.array([np.frombuffer(s, dtype=np.uint8) for s in next_states]), dtype=torch.float32).reshape(batch_size, -1, 1)
        dones = torch.tensor(dones, dtype=torch.float32)

        actions_pred, values, boundary_probs = self.model(states, states.shape[1])
        _, next_values, _ = self.model(next_states, next_states.shape[1])
        advantages = self.compute_gae(rewards, values.squeeze(-1), next_values.squeeze(-1), dones)

        # PPO loss 계산 (가우시안 정책으로 안정화)
        dist = torch.distributions.Normal(actions_pred, 0.1)  # Assume small stddev
        log_probs = dist.log_prob(actions).sum(dim=-1)
        old_dist = torch.distributions.Normal(actions, 0.1)
        old_log_probs = old_dist.log_prob(actions).sum(dim=-1)
        ratio = torch.exp(log_probs - old_log_probs)
        clipped_ratio = torch.clamp(ratio, 1 - self.clip_eps, 1 + self.clip_eps)
        action_loss = -torch.min(ratio * advantages, clipped_ratio * advantages).mean()
        value_loss = ((values.squeeze(-1) - rewards) ** 2).mean()
        ratio_loss = torch.mean(boundary_probs * torch.log(boundary_probs + 1e-10))
        loss = action_loss + 0.5 * value_loss + 0.1 * ratio_loss

        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=0.5)
        self.optimizer.step()
        self.last_loss = loss.item()  # 마지막 loss 저장

# --- 보상 함수 ---
def compute_reward(data):
    """보상 함수: x축 이동 속도 - 제어 비용 + 생존 보너스"""
    forward_velocity = data.qvel[0]
    ctrl_cost = 0.1 * np.sum(np.square(data.ctrl))
    z_pos = data.qpos[2]
    alive_bonus = 1.0 if z_pos > 0.3 else -1.0
    return forward_velocity - ctrl_cost + alive_bonus

# --- 학습 루프 ---
def train_hnet():
    hnet = HNet(d_model=512)
    ppo = PPO(hnet)
    num_episodes = 1000

    ckpt_dir = "./ckpt"
    if not os.path.exists(ckpt_dir):
        os.makedirs(ckpt_dir)

    episode_bar = trange(num_episodes, desc="Episode", position=0)
    for episode in episode_bar:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        state = get_state(data)
        episode_reward = 0
        states, actions, rewards = [], [], []

        step_bar = trange(max_steps, desc="Step", leave=False, position=1)
        for t in step_bar:
            state_bytes = float_to_byte_array(state, np.zeros(action_dim))
            state_tensor = torch.tensor(np.frombuffer(state_bytes, dtype=np.uint8), dtype=torch.float32).reshape(1, -1, 1)

            action, value, _ = hnet(state_tensor, state_tensor.shape[1])
            action = action.detach().numpy()
            if np.any(np.isnan(action)):
                print(f"[train_hnet] Warning: NaN detected in action at step {t}: {action}")
                action = np.zeros_like(action)  # Replace NaN with zeros

            data.ctrl[:] = np.clip(action, -1, 1)
            mujoco.mj_step(model, data)

            next_state = get_state(data)
            reward = compute_reward(data)
            done = (data.time >= max_steps * timestep) or (data.qpos[2] < 0.3)

            episode_reward += reward

            next_state_bytes = float_to_byte_array(next_state, action)
            ppo.store(state_bytes, action, reward, next_state_bytes, done)

            state = next_state
            states.append(state_bytes)
            actions.append(action)
            rewards.append(reward)

            # 렌더링
            if viewer:
                viewer.sync()
            elif renderer and video_writer:
                try:
                    renderer.update_scene(data)
                    frame = renderer.render()
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    video_writer.write(frame)
                except Exception as e:
                    print(f"[train_hnet] Rendering failed at step {t}: {e}")
                    print(traceback.format_exc())

            # tqdm에 현재 step, reward 표시
            step_bar.set_postfix({
                "step": t,
                "reward": f"{episode_reward:.2f}",
                "loss": f"{ppo.last_loss:.4f}" if ppo.last_loss is not None else "N/A"
            })

            if done:
                print(f"[train_hnet] 에피소드 {episode+1} 종료 (step {t})")
                break

        ppo.update()
        episode_bar.set_postfix({
            "episode_reward": f"{episode_reward:.2f}",
            "loss": f"{ppo.last_loss:.4f}" if ppo.last_loss is not None else "N/A"
        })

        if (episode + 1) % 100 == 0:
            save_path = os.path.join(ckpt_dir, f"hnet_ep{episode+1}.pt")
            torch.save(hnet.state_dict(), save_path)
            print(f"Model saved to {save_path}")

        if episode % 100 == 0:
            print("[train_hnet] 평가 시작")
            eval_reward = evaluate_hnet(hnet)
            print(f"Evaluation Reward: {eval_reward:.2f}")

# --- 평가 함수 ---
def evaluate_hnet(hnet):
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    state = get_state(data)
    total_reward = 0

    for t in range(max_steps):
        state_bytes = float_to_byte_array(state, np.zeros(action_dim))
        state_tensor = torch.tensor(np.frombuffer(state_bytes, dtype=np.uint8), dtype=torch.float32).reshape(1, -1, 1)
        action, _, _ = hnet(state_tensor, state_tensor.shape[1])
        action = action.detach().numpy()
        if np.any(np.isnan(action)):
            print(f"[evaluate_hnet] Warning: NaN detected in action at step {t}: {action}")
            action = np.zeros_like(action)

        data.ctrl[:] = np.clip(action, -1, 1)
        mujoco.mj_step(model, data)

        reward = compute_reward(data)
        total_reward += reward

        state = get_state(data)
        done = (data.time >= max_steps * timestep) or (data.qpos[2] < 0.3)

        if viewer:
            viewer.sync()
        elif renderer and video_writer:
            try:
                renderer.update_scene(data)
                frame = renderer.render()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                video_writer.write(frame)
            except Exception as e:
                print(f"[evaluate_hnet] Rendering failed at step {t}: {e}")
                print(traceback.format_exc())

        if done:
            print(f"[evaluate_hnet] 종료 (step {t})")
            break

    print("[evaluate_hnet] 완료")
    return total_reward

# --- 실행 ---
if __name__ == "__main__":
    print("[main] 시작")
    try:
        train_hnet()
    finally:
        if viewer:
            print("[main] Closing mujoco.viewer")
            viewer.close()
        if renderer and video_writer:
            print("[main] Closing renderer and video writer")
            renderer.close()
            video_writer.release()