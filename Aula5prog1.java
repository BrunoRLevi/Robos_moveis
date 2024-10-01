import lejos.nxt.*;
import lejos.util.Delay;
import lejos.robotics.navigation.*;
import lejos.robotics.mapping.*;
import lejos.robotics.pathfinding.*;
import lejos.robotics.localization.*;


import lejos.geom.*;

public class Aula5prog1 {
  public static void main(String[] args) {
    try {
      // Exibe uma mensagem para indicar que o programa está em execução
      System.out.println("Running");
      // Aguarda o usuário pressionar qualquer botão para iniciar
      Button.waitForAnyPress();
      // Introduz uma pausa de 500 milissegundos antes de iniciar o movimento
      Delay.msDelay(500);

      // Inicializa o DifferentialPilot para controlar os motores do robô
      DifferentialPilot pilot = new DifferentialPilot(2.205 * 2.56, 4.527 * 2.56, Motor.A, Motor.B, false);
      // Define a velocidade de deslocamento linear para 15 cm/s
      pilot.setTravelSpeed(15);
      // Define a velocidade de rotação para 30 graus/s
      pilot.setRotateSpeed(30);

      // Provedor de Pose baseado em Odometria que calcula a posição e orientação do
      // robô
      OdometryPoseProvider poseProvider = new OdometryPoseProvider(pilot);

      // Cria um objeto Navigator para controlar a navegação do robô
      Navigator navigator = new Navigator(pilot);

      int erro = 5;
      // Define um conjunto de linhas que representarão obstáculos no mapa
      Line[] lines = {
          // school
          new Line(30 - erro, -1000, 30 - erro, 30 + erro + 1),
          new Line(30 - erro, 30 + erro, 44.5f, 30 + erro),
          new Line(45.5f, 30 + erro, 60 + erro, 30 + erro),
          new Line(60 + erro, 30 + erro, 60 + erro, 15.5f),
          new Line(60 + erro, 14.5f, 60 + erro, -1000),

          // bakery
          new Line(90 - erro, -1000, 90 - erro, 30 + erro + 1),
          new Line(90 - erro, 30 + erro, 104.5f, 30 + erro),
          new Line(105.5f, 30 + erro, 120 + erro, 30 + erro),
          new Line(120 + erro, 30 + erro, 120 + erro, 15.5f),
          new Line(120 + erro, 14.5f, 120 + erro, -1000),

          // city hall
          new Line(30 - erro, 60 - erro - 1, 30 - erro, 90 + erro + 1),
          new Line(30 - erro, 90 + erro, 44.5f, 90 + erro),
          new Line(45.5f, 90 + erro, 60 + erro, 90 + erro),
          new Line(60 + erro, 90 + erro + 1, 60 + erro, 60 - erro - 1),
          new Line(60 + erro, 60 - erro, 45.5f, 60 - erro),
          new Line(44.5f, 60 - erro, 30 - erro, 60 - erro),

          // drug store
          new Line(90 -erro -1, 60-erro, 120+erro+1, 60-erro),
          new Line(120+erro, 60-erro, 120+erro, 74.5f), 
          new Line(120 + erro, 75.5f, 120 + erro, 90 + erro), 
          new Line(120 + erro+1, 90 +erro, 90 -erro -1, 90+erro), 
          new Line(90 -erro, 90+ erro, 90 -erro , 75.5f), 
          new Line(90 -erro, 60 -erro, 90 -erro, 74.5f), 

          // museum
          new Line(90 - erro, 120 - erro, 104.5f, 120 - erro),
          new Line(105.5f, 120 - erro, 120 +erro, 120 -erro),
          new Line(120 + erro, 120- erro -1, 120 + erro, 1000),
          new Line(120 + erro, 1000, 90 - erro, 1000),
          new Line(90 - erro, 1000, 90 -erro, 135.5f),
          new Line(90 - erro, 134.5f, 90 - erro, 120-erro -1),

          // library
          new Line(30 - erro, 1000, 30 - erro, 135.5f),
          new Line(30 - erro, 134.5f, 30 - erro, 120 - erro),
          new Line(30 - erro, 120 - erro - 1, 44.5f, 120 - erro),
          new Line(45.5f, 120 - erro, 60 + erro, 120 - erro),
          new Line(60 + erro, 120 - erro - 1, 60 + erro, 1000)
      };

      Rectangle bounds = new Rectangle(0, 0, 178, 150);

      // Cria um mapa de linhas (LineMap) que inclui os obstáculos e os limites do
      // mapa
      LineMap map = new LineMap(lines, bounds);

      // Inicializa o algoritmo de busca de caminho (pathfinding) usando o algoritmo
      // de Dijkstra
      DijkstraPathFinder pathFinder = new DijkstraPathFinder(map);

      float[][] mapping = { { 45f, 15f }, { 105f, 15f }, { 45f, 75f }, { 105f, 75f }, { 45f, 135f }, { 105f, 135f } };

      int mapping_len = 6;
      float[] wp_coord = null;
      int i = 0;
      while (true) {
        System.out.print(mapping[i % 6][0]);
        System.out.print(" ");
        System.out.println(mapping[i % 6][1]);

        Button.waitForAnyPress();

        // if a button other than down button is pressed
        if (Button.readButtons() != 8) {
          wp_coord = mapping[i % 6];
          break;
        }

        i++;
      }

      System.out.print("Going to waypoint = ");
      System.out.print(wp_coord[0]);
      System.out.print(" ");
      System.out.println(wp_coord[1]);

      Waypoint wp = new Waypoint(wp_coord[0], wp_coord[1]);

      // Calcula a rota do ponto de partida (pose atual) até o waypoint usando o
      // algoritmo de Dijkstra
      navigator.setPath(pathFinder.findRoute(poseProvider.getPose(), wp));

      // Faz o robô seguir o caminho calculado
      navigator.followPath();

      Pose curr_pose = poseProvider.getPose();

      if (curr_pose.getY() > 90) {
        navigator.addWaypoint(0, 105);
      }
      navigator.addWaypoint(0, 45);

      // Aguarda o usuário pressionar qualquer botão antes de encerrar o programa
      // Button.waitForAnyPress();
      Button.waitForAnyPress();
    } catch (Exception e) {
      // Imprime qualquer exceção que ocorra durante a execução
      System.out.println(e);
    }
  }
}