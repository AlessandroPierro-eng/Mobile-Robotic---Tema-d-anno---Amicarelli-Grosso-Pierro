#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# Messaggi per Nav2 e ROS
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped

# Librerie per le Trasformate (TF2)
import tf2_ros
from tf2_ros import TransformException

import tf2_geometry_msgs  # Indispensabile per applicare la trasformata ai punti


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        self.get_logger().info("Inizializzazione Robot Manager in corso...")

        # ==========================================
        # 1. MEMORIA DELLA MACCHINA A STATI (FSM)
        # ==========================================
        self.state = 'PATROL'  # Stati: PATROL, CHASE, SEARCH, INTERCEPT
        
        # Variabili di stato per l'Intruso
        self.intruder_seen_recently = False
        self.last_intruder_time = 0.0
        self.last_intruder_pose = None  # Salveremo qui le coordinate globali (X, Y)
        
        # Variabili per la Ronda (Waypoints)
        # Esempio di 2 punti mappa [X, Y]. Li modificheremo con i tuoi veri waypoint.
        self.waypoints = [[7.7, 4.0], [-19.5, 6.1], [-20.5, -6.5], [6.8, -8.3]]
        self.current_wp_index = 0
        
        # ==========================================
        # 2. SISTEMA DI TRASFORMATE (TF2)
        # ==========================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ==========================================
        # 3. INTERFACCE DI COMUNICAZIONE (Muscoli e Sensi)
        # ==========================================

        # Action Client per comandare Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None     # Memorizza il viaggio attuale per poterlo cancellare
        self.is_navigating = False      # Bandierina per sapere se il robot si sta muovendo
        
        # Sottoscrizione a YOLO (Il nostro Occhio)
        self.yolo_sub = self.create_subscription(
            PointStamped,
            '/intruder_tracking',
            self.yolo_callback,
            10)
            
        # Sottoscrizione agli Ordini Globali (Lo Sciame) - Per il futuro
        self.global_order_sub = self.create_subscription(
            PointStamped,
            '/global_orders',
            self.global_order_callback,
            10)

        # Editore per l'Allarme Globale
        self.alarm_pub = self.create_publisher(PointStamped, '/global_intruder_alert', 10)    

        # ==========================================
        # 4. IL BATTITO CARDIACO (Timer del Control Loop)
        # ==========================================
        timer_period = 0.2  # 5 Hz (esegue il loop 5 volte al secondo)
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info("Robot Manager PRONTO. Stato iniziale: PATROL")

    # -------------------------------------------------------------------------
    # CALLBACKS (I SENSI - Aggiornano solo le bandierine, NON prendono decisioni)
    # -------------------------------------------------------------------------
    def yolo_callback(self, msg):
        """Riceve i dati da YOLO e aggiorna la memoria a breve termine."""
        if msg.point.z > 0.0:
            # L'intruso è visibile!
            self.intruder_seen_recently = True
            self.last_intruder_time = time.time()
            
            # 1. TRASFORMAZIONE IN COORDINATE MAPPA
            global_coords = self.get_global_pose(msg)
            if global_coords:
                self.last_intruder_pose = global_coords
                
                # 2. ALLARME GLOBALE
                # Creiamo il messaggio da urlare allo sciame
                alarm_msg = PointStamped()
                alarm_msg.header.stamp = self.get_clock().now().to_msg()
                # Usiamo il frame_id per "firmare" il messaggio (utile per il Global Manager)
                alarm_msg.header.frame_id = self.get_name() 
                alarm_msg.point.x = global_coords[0]
                alarm_msg.point.y = global_coords[1]
                alarm_msg.point.z = 0.0
                
                self.alarm_pub.publish(alarm_msg)
        else:
            # YOLO ha perso l'intruso (Z = -1.0)
            self.intruder_seen_recently = False

    def get_global_pose(self, local_point_msg):
        """Trasforma un punto dal sistema della telecamera a quello della mappa globale."""
        try:
            # Chiediamo a TF2 la relazione spaziale tra la 'map' e la telecamera in questo esatto istante
            transform = self.tf_buffer.lookup_transform(
                'map',
                local_point_msg.header.frame_id,
                rclpy.time.Time(), # Prendi la trasformata più recente
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Applichiamo la trasformazione al punto
            global_point_msg = tf2_geometry_msgs.do_transform_point(local_point_msg, transform)
            
            # Estraiamo solo X e Y (il robot si muove sul piano 2D)
            return [global_point_msg.point.x, global_point_msg.point.y]
            
        except TransformException as ex:
            self.get_logger().error(f"Errore TF2: Impossibile localizzare l'intruso sulla mappa. Dettagli: {ex}")
            return None

    # -------------------------------------------------------------------------
    # IL CERVELLO (Valuta le transizioni e gestisce le azioni)
    # -------------------------------------------------------------------------
    def control_loop(self):
        """Viene eseguito 5 volte al secondo. Gestisce la FSM."""
        current_time = time.time()
        
        # --- A. VALUTAZIONE DELLE TRANSIZIONI (Chi vince?) ---
        if self.intruder_seen_recently:
            if self.state != 'CHASE':
                self.get_logger().warn("INTRUSO AVVISTATO! Transizione a CHASE.")
            self.state = 'CHASE'
            
        elif self.state == 'CHASE' and not self.intruder_seen_recently:
            # Abbiamo appena perso il bersaglio
            time_since_lost = current_time - self.last_intruder_time
            if time_since_lost < 10.0:
                if self.state != 'SEARCH':
                    self.get_logger().info("Bersaglio perso. Inizio procedura SEARCH...")
                self.state = 'SEARCH'
            else:
                self.get_logger().info("Bersaglio sparito da troppo tempo. Torno in PATROL.")
                self.state = 'PATROL'

        # --- B. ESECUZIONE DELLO STATO ATTUALE ---
        if self.state == 'PATROL':
            # Se il robot è fermo, mandiamolo al prossimo waypoint
            if not self.is_navigating:
                wp = self.waypoints[self.current_wp_index]
                self.get_logger().info(f"Ronda: Dirigo al WP {self.current_wp_index} -> {wp}")
                self.send_nav_goal(wp[0], wp[1])
                
                # Passa al prossimo waypoint (ciclico)
                self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
                
        elif self.state == 'CHASE':
            self.is_navigating = True
            # Se stiamo inseguendo, andiamo verso il ladro
            if self.last_intruder_pose:
                target_x = self.last_intruder_pose[0]
                target_y = self.last_intruder_pose[1]
                
                # Calcoliamo a spanne la distanza tra dove stiamo andando ora e dove è il ladro
                # Se non abbiamo un goal attivo o se il ladro si è mosso di oltre 0.5 metri, aggiorniamo la rotta
                import math
                if not hasattr(self, 'current_chase_target'):
                    self.current_chase_target = [0, 0]
                    
                distanza_spostamento = math.hypot(target_x - self.current_chase_target[0], 
                                                  target_y - self.current_chase_target[1])
                                                  
                if not self.is_navigating or distanza_spostamento > 0.5:
                    self.current_chase_target = [target_x, target_y]
                    # self.send_nav_goal(target_x, target_y)
                    self.send_nav_goal(-20.5, -6.5)
            
        elif self.state == 'SEARCH':
            # Rimuovi (o commenta) queste righe!
            # if self.is_navigating:
            #     self.get_logger().info("Cancellazione del goal Nav2 corrente...")
            #     self.cancel_nav_goal()
            
            # Al loro posto, metti semplicemente 'pass', così Nav2 continua a guidare
            pass

# -------------------------------------------------------------------------
    # I MUSCOLI (Controllo di Nav2)
    # -------------------------------------------------------------------------
    def send_nav_goal(self, x, y):
        """Invia un obiettivo a Nav2."""
        # Aspetta che il server Nav2 sia pronto
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server non disponibile!")
            return

        # Crea il messaggio PoseStamped (posizione del ladro in cordinate assolute)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientamento (Quaternione). Mettiamo un valore neutro (guarda dritto)
        goal_msg.pose.pose.orientation.w = 1.0 

        self.get_logger().info(f"Invio target Nav2 -> X: {x:.2f}, Y: {y:.2f}")
        self.is_navigating = True
        
        # Invia l'azione in modo asincrono (per non bloccare il control_loop)
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_nav_goal(self):
        """Interrompe la navigazione corrente (Freno a mano)."""
        if self.nav_goal_handle is not None and self.is_navigating:
            self.get_logger().info("Cancellazione del goal Nav2 corrente...")
            self.nav_goal_handle.cancel_goal_async()
            self.is_navigating = False

    # -- Callback interne di Nav2 (Servono per sapere se il goal è stato accettato o completato) --
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rifiutato da Nav2!")
            self.is_navigating = False
            return
        
        self.nav_goal_handle = goal_handle
        # Chiediamo a Nav2 di avvisarci quando ha finito
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.is_navigating = False
        self.get_logger().info(f"Navigazione completata con stato: {status}")

     # -- Callback per il global manager (DA DEFINIRE DOPO) --

    def global_order_callback(self, msg):
        """Riceve ordini di accerchiamento dal Global Manager (Da implementare)."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()