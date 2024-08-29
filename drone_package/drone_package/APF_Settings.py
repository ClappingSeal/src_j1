import numpy as np

# bbbbbbb
class APFEnvironment:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)
        self.limit = 3
        self.observation_radius = 1000

    # state, apf
    def att_force(self, goal, att_gain=1):
        goal_vector = np.array(goal) - self.pos
        norm = np.linalg.norm(goal_vector)
        if norm == 0:
            return np.zeros_like(goal_vector)
        return goal_vector / norm * att_gain

    # state, apf
    def rep_force(self, obs_info, rep_gain=1):
        force = np.zeros(2)

        for obs in obs_info:
            x, y, r = obs
            distance_vector = self.pos - np.array([x, y])
            distance = np.linalg.norm(distance_vector) - r

            if distance == 0:
                print('error')
                continue
            if (self.limit < distance) and (distance < self.observation_radius):
                repulsive_force_magnitude = rep_gain * (1 / (distance - self.limit))
                repulsive_force_direction = distance_vector / distance
                force += repulsive_force_magnitude * repulsive_force_direction
            elif distance <= self.limit:
                repulsive_force_magnitude = rep_gain * 200
                repulsive_force_direction = distance_vector / distance
                force += repulsive_force_magnitude * repulsive_force_direction

        return force

    # state
    def heuristic(self, goal):
        goal = np.array(goal)
        return goal - self.pos

    # rev_obs_center
    def within_obs(self, obs_info):
        obs_pos = [np.array(pos[:2]) for pos in obs_info]
        within_radius = [pos for pos in obs_pos if np.linalg.norm(pos - self.pos) <= self.observation_radius]

        if not within_radius:
            within_radius = [np.array([self.pos[0], self.pos[1]])]  # 로봇의 현재 위치를 포함하는 더미 항목 추가

        return within_radius

    # state
    def closest_obs(self, obs_info):
        obs_pos = [np.array(pos[:2]) for pos in obs_info]
        radius = [np.array(pos[2:]) for pos in obs_info]
        radius = np.array([r[0] for r in radius])

        if not obs_pos:
            return None
        distances = [np.linalg.norm(self.pos - pos) for pos in obs_pos] - radius

        closest_index = np.argmin(distances)
        closest_pos = obs_pos[closest_index]
        return closest_pos - self.pos

    # func, env
    def apf(self, goal, obs_info):
        total_force = self.att_force(goal) + self.rep_force(obs_info)
        return total_force / np.linalg.norm(total_force)

    # func, action
    def apf_rev_rotate(self, goal, obs_info):

        # 1. 기본 변수랑 각도 구하기
        goal = np.array(goal)
        apf_vector = self.apf(goal, obs_info)

        angle = np.pi / 2 - np.arctan2(apf_vector[1], apf_vector[0])
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])

        # 2. 함수 계산
        rev_att_vector = self.att_force(goal)
        rev_rep_vector = self.rep_force(obs_info)
        closest_obs_pos = self.closest_obs(obs_info)

        # 3. 회전
        rot_rev_att_vector = np.dot(rotation_matrix, rev_att_vector.T)
        rot_rev_rep_vector = np.dot(rotation_matrix, rev_rep_vector.T)

        if closest_obs_pos is not None:
            rev_closest_obs_pos = closest_obs_pos - self.pos
            rot_rev_closest_obs_pos = np.dot(rotation_matrix, rev_closest_obs_pos.T)
        else:
            rot_rev_closest_obs_pos = np.array([0, 0])

        return rot_rev_att_vector, rot_rev_rep_vector, rot_rev_closest_obs_pos

    # func, action
    def apf_inverse_rotate(self, goal, obs, b_vector):
        goal = np.array(goal)
        apf_vector = self.apf(goal, obs)

        angle = np.pi / 2 - np.arctan2(apf_vector[1], apf_vector[0])
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        inverse_rotation_matrix = rotation_matrix.T
        inverse_rotated_b_vector = np.dot(inverse_rotation_matrix, b_vector)

        return inverse_rotated_b_vector

    # action
    def apf_drl(self, goal, obs_info, a, b):
        total_force = 2 * (a * self.att_force(goal) + (1 - a) * self.rep_force(obs_info)) + b
        return total_force / np.linalg.norm(total_force)

# env = APFEnv([1, 3])
# print(env.att_force([10, 4]))
# print(env.rep_force([[1, 1, 0.9]]))
# print(env.apf([10, 4], [[2, 3, 1], [1, 6, 1]]))
# print(env.closest_obs([[100, 101, 50], [100, 100, 1]]))
