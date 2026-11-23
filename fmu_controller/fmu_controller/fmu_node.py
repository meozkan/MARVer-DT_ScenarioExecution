import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import fmpy
import time
import pandas as pd
from ament_index_python.packages import get_package_share_directory

class FMUControllerNode(Node):
    def __init__(self):
        super().__init__('fmu_controller_node')
        
        # --- Ayarlar ---
        self.t = 0.0
        self.dt = 0.001
        self.target_position = 1.0
        self.results = []
        
        # Manuel Sönümleme Katsayısı (FMU salınımı durduramazsa bunu kullanacağız)
        # Eğer salınım çoksa bu değeri 0.5 - 2.0 arasına çıkarın
        self.manual_damping = 0

        # --- FMU Hazırlığı ---
        fmu_name = 'RotarySingleRod_controller_SemiImplicit.fmu'

        try:
            package_share_dir = get_package_share_directory('fmu_controller')
            fmu_filename = os.path.join(package_share_dir, fmu_name)
            if not os.path.exists(fmu_filename):
                raise FileNotFoundError(f"Share klasöründe bulunamadı: {fmu_filename}")
        except Exception as e:
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
        self.fmu_instance.setupExperiment(startTime=0.0)
        self.fmu_instance.enterInitializationMode()
        self.fmu_instance.exitInitializationMode()

        # --- İletişim ---
        # Kuyruk derinliğini (History Policy) 1 yaparak en taze veriyi alıyoruz
        self.subscription = self.create_subscription(
            JointState,
            '/world/RotarySingleRod_world/model/RotarySingleRod_model/joint_state',
            self.joint_state_callback,
            1) 
        
        self.publisher = self.create_publisher(Float64, '/model/RotarySingleRod_model/joint/rod_joint/cmd_force', 10)

        self.get_logger().info("FMU ROS 2 Controller Başlatıldı. 'rod_joint' dinleniyor...")

    def joint_state_callback(self, msg):
        start_time = time.perf_counter()

        # 1. Doğru eklemi (rod_joint) isme göre bul
        try:
            idx = msg.name.index('rod_joint')
        except ValueError:
            return # rod_joint bulunamadıysa işlem yapma

        current_pos = msg.position[idx]
        current_vel = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
        
        # 2. Hata hesapla
        epsilon = self.target_position - current_pos

        # 3. FMU Hesaplama
        if 'epsilon' in self.v_refs:
            self.fmu_instance.setReal([self.v_refs['epsilon']], [epsilon])
        
        self.fmu_instance.doStep(currentCommunicationPoint=self.t, communicationStepSize=self.dt)

        if 'tau' in self.v_refs:
            tau_output = self.fmu_instance.getReal([self.v_refs['tau']])[0]
            
            # --- EKSTRA SÖNÜMLEME (Salınımı Durdurmak İçin) ---
            # Eğer FMU torku sistemi sallıyorsa, hıza zıt bir tork ekliyoruz
            final_tau = tau_output #- (self.manual_damping * current_vel)
            
            # 4. Torku Yayınla
            cmd_msg = Float64()
            cmd_msg.data = float(final_tau)
            self.publisher.publish(cmd_msg)
            
            # Veri Kaydı
            self.results.append([self.t, final_tau, current_pos])
        
        self.t += self.dt

        # Zamanlama Uyarısı
        duration = time.perf_counter() - start_time
        if duration > self.dt:
            self.get_logger().warning(f"Gecikme: {duration:.4f}s > {self.dt}s")

    def save_data(self):
        if self.results:
            df = pd.DataFrame(self.results, columns=['Zaman', 'Tork', 'Pozisyon'])
            df.to_csv('FMU_ROSnode_data.csv', index=False)
            self.get_logger().info(f"Veriler 'FMU_ROSnode_data.csv' dosyasına kaydedildi.")
        
        self.fmu_instance.terminate()
        self.fmu_instance.freeInstance()
        fmpy.util.cleanup(self.unzip_dir)

def main(args=None):
    rclpy.init(args=args)
    node = FMUControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Durduruluyor...")
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
