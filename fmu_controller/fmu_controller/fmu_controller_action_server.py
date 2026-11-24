import os
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import fmpy
import pandas as pd
from ament_index_python.packages import get_package_share_directory

from fmu_controller_interfaces.action import MoveToPosition


class FMUControllerActionServer(Node):
    def __init__(self):
        super().__init__('fmu_controller_action_server')

        # --- Parametreler ---
        self.declare_parameter('dt', 0.001)
        self.declare_parameter('position_tolerance', 0.01)   # rad
        self.declare_parameter('velocity_tolerance', 0.05)   # rad/s
        self.declare_parameter('settle_time', 0.5)           # s (toleransta kalma süresi)
        self.declare_parameter('timeout', 30.0)              # s
        self.declare_parameter('feedback_rate', 20.0)        # Hz

        self.dt = self.get_parameter('dt').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.vel_tol = self.get_parameter('velocity_tolerance').value
        self.settle_time = self.get_parameter('settle_time').value
        self.timeout = self.get_parameter('timeout').value
        self.feedback_rate = self.get_parameter('feedback_rate').value

        # --- Durum (callback'ler arası paylaşılan) ---
        self.lock = threading.Lock()
        self.t = 0.0
        self.target_position = 0.0
        self.active = False              # kontrol döngüsü çalışıyor mu
        self.current_pos = float('nan')
        self.current_vel = 0.0
        self.results = []

        # --- FMU Hazırlığı ---
        fmu_name = 'RotarySingleRod_controller_SemiImplicit.fmu'
        
        try:
            package_share_dir = get_package_share_directory('fmu_controller')
            fmu_filename = os.path.join(package_share_dir, fmu_name)
            if not os.path.exists(fmu_filename):
                raise FileNotFoundError(f"Share klasöründe bulunamadı: {fmu_filename}")
        except Exception:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            fmu_filename = os.path.join(script_dir, fmu_name)

        self.get_logger().info(f"FMU Yükleniyor: {fmu_filename}")

        if not os.path.exists(fmu_filename):
            self.get_logger().error(f"FMU DOSYASI BULUNAMADI! Aranan yol: {fmu_filename}")
            raise FileNotFoundError(f"FMU dosyası yok: {fmu_filename}")

        self.unzip_dir = fmpy.extract(fmu_filename)

        model_description = fmpy.read_model_description(self.unzip_dir)
        self.v_refs = {v.name: v.valueReference for v in model_description.modelVariables}

        self.fmu_instance = fmpy.instantiate_fmu(self.unzip_dir, model_description, 'CoSimulation')
        self._init_fmu()

        # --- İletişim ---
        # Joint state callback'i (1 kHz) ile action execute döngüsü aynı anda
        # çalışabilsin diye Reentrant callback group + MultiThreadedExecutor kullanılır.
        self.cb_group = ReentrantCallbackGroup()

        #Create a subscriber for the joint state topic with a queue size of 1 to get the latest data 
        self.subscription = self.create_subscription(
            JointState,
            '/world/RotarySingleRod_world/model/RotarySingleRod_model/joint_state',
            self.joint_state_callback,
            1,
            callback_group=self.cb_group)

        #Create a publisher for the command force topic with a queue size of 10
        self.publisher = self.create_publisher(
            Float64, '/model/RotarySingleRod_model/joint/rod_joint/cmd_force', 10)

        #Create an action server for the MoveToPosition action with the execute, goal, and cancel callbacks
        self._action_server = ActionServer(
            self,
            MoveToPosition,
            'move_to_position',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        self.get_logger().info("FMU Action Server hazır: 'move_to_position'")

    # ------------------------------------------------------------------ FMU
    def _init_fmu(self):
        self.fmu_instance.setupExperiment(startTime=0.0)
        self.fmu_instance.enterInitializationMode()
        self.fmu_instance.exitInitializationMode()

    def _reset_fmu(self):
        # Her yeni hedefte kontrolcü iç durumunu (integratör vb.) sıfırla
        self.fmu_instance.reset()
        self._init_fmu()
        self.t = 0.0

    # --------------------------------------------------------- Kontrol döngüsü
    def joint_state_callback(self, msg):
        start_time = time.perf_counter()

         # 1. Doğru eklemi (rod_joint) isme göre bul
        try:
            idx = msg.name.index('rod_joint')
        except ValueError:
            return  # rod_joint bulunamadıysa işlem yapma

        current_pos = msg.position[idx]
        current_vel = msg.velocity[idx] if len(msg.velocity) > idx else 0.0

        with self.lock:
            self.current_pos = current_pos
            self.current_vel = current_vel

            if not self.active:
                return
            # 2. Hata hesapla
            epsilon = self.target_position - current_pos

            # 3. FMU Hesaplama
            if 'epsilon' in self.v_refs:
                self.fmu_instance.setReal([self.v_refs['epsilon']], [epsilon])

            self.fmu_instance.doStep(currentCommunicationPoint=self.t,
                                     communicationStepSize=self.dt)

            if 'tau' in self.v_refs:
                tau = self.fmu_instance.getReal([self.v_refs['tau']])[0]
                # 4. Torku Yayınla
                cmd_msg = Float64()
                cmd_msg.data = float(tau)
                self.publisher.publish(cmd_msg)
                # Veri Kaydı
                self.results.append([self.t, tau, current_pos, self.target_position])

            self.t += self.dt

        # Zamanlama Uyarısı
        duration = time.perf_counter() - start_time
        if duration > self.dt:
            self.get_logger().warning(f"Gecikme: {duration:.4f}s > {self.dt}s")

    def _stop_control(self):
        with self.lock:
            self.active = False
        # Kontrol bittiğinde tork komutunu sıfırla
        self.publisher.publish(Float64(data=0.0))

    # ------------------------------------------------------------ Action callbacks
    def goal_callback(self, goal_request):
        with self.lock:
            if self.active:
                self.get_logger().warning("Zaten aktif bir hedef var, yeni hedef reddedildi.")
                return GoalResponse.REJECT
        self.get_logger().info(f"Hedef alındı: {goal_request.target_position:.4f}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("İptal isteği alındı.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_position

        with self.lock:
            self._reset_fmu()
            self.target_position = target
            self.active = True

        feedback = MoveToPosition.Feedback()
        result = MoveToPosition.Result()
        period = 1.0 / self.feedback_rate
        t_start = time.monotonic()
        settle_start = None

        while rclpy.ok():
            # İptal
            if goal_handle.is_cancel_requested:
                self._stop_control()
                goal_handle.canceled()
                with self.lock:
                    result.final_position = self.current_pos
                result.success = False
                self.get_logger().info("Hedef iptal edildi.")
                return result

            with self.lock:
                pos = self.current_pos
                vel = self.current_vel

            # Henüz joint_state gelmediyse beklemeye devam et
            if not math.isnan(pos):
                error = target - pos
                feedback.position_error = float(error)
                goal_handle.publish_feedback(feedback)

                # Yakınsama: konum hatası ve hız tolerans içinde, settle_time boyunca
                if abs(error) < self.pos_tol and abs(vel) < self.vel_tol:
                    if settle_start is None:
                        settle_start = time.monotonic()
                    elif time.monotonic() - settle_start >= self.settle_time:
                        self._stop_control()
                        result.final_position = float(pos)
                        result.success = True
                        goal_handle.succeed()
                        self.get_logger().info(f"Hedefe ulaşıldı. Konum: {pos:.4f}")
                        return result
                else:
                    settle_start = None

            # Zaman aşımı
            if time.monotonic() - t_start > self.timeout:
                self._stop_control()
                result.final_position = float(pos) if not math.isnan(pos) else 0.0
                result.success = False
                goal_handle.abort()
                self.get_logger().error("Zaman aşımı, hedef iptal edildi (abort).")
                return result

            time.sleep(period)

        # rclpy kapanıyorsa
        self._stop_control()
        result.final_position = float(self.current_pos)
        result.success = False
        goal_handle.abort()
        return result

    # ------------------------------------------------------------------ Kapanış
    def save_data(self):
        if self.results:
            df = pd.DataFrame(self.results, columns=['Zaman', 'Tork', 'Pozisyon', 'Hedef'])
            df.to_csv('FMU_ROSnode_data.csv', index=False)
            self.get_logger().info("Veriler 'FMU_ROSnode_data.csv' dosyasına kaydedildi.")

        self.fmu_instance.terminate()
        self.fmu_instance.freeInstance()
        fmpy.util.cleanup(self.unzip_dir)


def main(args=None):
    rclpy.init(args=args)
    node = FMUControllerActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Durduruluyor...")
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()