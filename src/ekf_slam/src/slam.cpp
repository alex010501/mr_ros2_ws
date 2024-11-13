#include "slam.h"
#include <angles/angles.h>
#include <sstream>
#include <math.h>


void Slam::on_odo(const nav_msgs::Odometry& odom)
{
  v = odom.twist.twist.linear.x;
  w = odom.twist.twist.angular.z;
}

void Slam::add_landmark(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish)
{
  // Добавляем только особые точки, на которые попало более 2 лучей
  if (finish - start < 2) {
    return;
  }
  // TODO Здесь должен быть код определения координаты центра круглого препятствия 

  double x = 0.0, y = 0.0;

  // Вычисляем среднее расстояние и угол для определения центра круглого препятствия
  double pointDist = 0.0;
  double pointAngle = 0.0;
  int total = 0;

  for (size_t idx = start; idx < finish; ++idx) {
    if (scan.ranges[idx] >= scan.range_max) {
      continue;
    }
    pointAngle += idx * scan.angle_increment - M_PI_2;
    pointDist += scan.ranges[idx];
    total++;
  }

  // Рассчитываем средний угол и расстояние
  pointAngle /= total;
  pointDist /= total;
  pointDist += feature_rad; // Добавляем радиус для получения центра круглого препятствия

  // Вычисляем координаты особой точки относительно лазера
  x = pointDist * cos(pointAngle);
  y = pointDist * sin(pointAngle);

  // Добавляем координаты особой точки в вектор особых точек
  new_landmarks.push_back(Eigen::Vector2d(x, y));
  new_landmarks_measurement.push_back(Eigen::Vector2d(pointDist, pointAngle));
}

void Slam::detect_landmarks(const sensor_msgs::LaserScan& scan)
{
  new_landmarks.clear();
  new_landmarks_measurement.clear();

  // TODO Здесь должен быть код для пределения особенные точек скана
  // ищем начальный и конечный индексы лучей, падающих на одно препятствие
  // и вызываем add_landmark

  size_t index = 0;

  while (index < scan.ranges.size()) {
    // Пропускаем значения, которые превышают максимальное расстояние
    if (scan.ranges[index] > scan.range_max - 1.0) {
      ++index;
    }
    // Если разница между текущим и следующим лучом меньше порога feature_rad,
    // считаем это за начало объекта, далее ищем его окончание
    else if (std::abs(scan.ranges[index] - scan.ranges[index + 1]) < feature_rad) {
      size_t end_index = index + 1;

      while (std::abs(scan.ranges[end_index] - scan.ranges[end_index + 1]) < feature_rad &&
             end_index < scan.ranges.size() - 1) {
        ++end_index;
      }

      // Вызываем add_landmark для обнаруженного объекта
      add_landmark(scan, index, end_index);
      index = end_index + 1;
    }
    else {
      // Если разница между текущим и следующим лучом больше порога feature_rad,
      // продолжаем поиск следующего объекта
      ++index;
    }

    // Проверка на выход за границы массива
    if (index >= scan.ranges.size()) {
      break;
    }
  }
}

int Slam::associate_measurement(const Eigen::Vector2d& landmark_measurement) {
  double nearest_distance = 1e10;
  int nearest_index = -1;
  // преобразование от СК карты к СК робота (дальномера)
  Eigen::Isometry2d robot_to_map = Eigen::Translation2d(X.segment(0, 2))
                                 * Eigen::Rotation2Dd(X(2));
  for (std::size_t i = 0; i < landmarks_found_quantity; ++i) {
    double distance = (robot_to_map * landmark_measurement - X.segment(ROBOT_STATE_SIZE + i*2, 2)).norm();
    if (distance < nearest_distance) {
      nearest_index = i;
      nearest_distance = distance;
    }
  }
  // naive association
  const double kAssocThreshold = 5.0;
  if (nearest_index >= 0 && nearest_distance < kAssocThreshold) {
    return nearest_index;
  }
  return -1;
}

int Slam::add_landmark_to_state(int measurementIndex)
{
  ++landmarks_found_quantity;

  // TODO init landmark in state
  // Здесь должен быть код по инициализации части вектора состояния, соответствующей 
  // маяку с индексом last_found_landmark_index

  // Получаем измерение маяка из вектора новых маяков
  Eigen::Vector2d landmark_measurement = new_landmarks[measurementIndex];
  Eigen::Vector2d newLandmark;

  // Делаем преобразование из системы координат карты в систему координат робота
  Eigen::Isometry2d robot_to_map = Eigen::Translation2d(X.segment(0, 2))
                                 * Eigen::Rotation2Dd(X(2));
  newLandmark = robot_to_map * landmark_measurement;

  // Обновляем состояния для нового маяка
  X[ROBOT_STATE_SIZE + 2 * landmarks_found_quantity - 2] = newLandmark[0];
  X[ROBOT_STATE_SIZE + 2 * landmarks_found_quantity - 1] = newLandmark[1];

  int landmarkIndex = landmarks_found_quantity;
  double x = X(0);
  double y = X(1);
  double theta = X(2);

  // Записываем результат измерения
  double zi_r = new_landmarks_measurement[measurementIndex](0);
  double zi_phi = new_landmarks_measurement[measurementIndex](1);

  // Записываем результат ковариации (мера взаимосвязи двух случайных величин,
  // измеряющая общее отклонение двух случайных величин от их ожидаемых значений.
  // Метрика оценивает, в какой степени переменные изменяются вместе.)
  double pmxx = P(0, 0);
  double pmxy = P(0, 1);
  double pmxt = P(0, 2);
  double pmyy = P(1, 1);
  double pmyt = P(1, 2);
  double pmtt = P(2, 2);

  double dr = Q(0, 0);
  double dphi = Q(0, 0);

  double s_zt = sin(zi_phi + theta);
  double c_zt = cos(zi_phi + theta);

  Eigen::Matrix2d PLi;
  PLi.setZero();

  // Рассчитываем значения из неопределенности движения
  PLi(0, 0) += pmxx - 2 * zi_r * s_zt * pmxt + pow(zi_r * s_zt, 2.) * pmtt;
  PLi(0, 1) += pmxy - zi_r * s_zt * pmyt + zi_r * c_zt * pmxt - zi_r * zi_r * s_zt * c_zt * pmtt;
  PLi(1, 0) = PLi(0, 1);
  PLi(1, 1) += pmyy + 2 * zi_r * c_zt * pmyt + pow(zi_r * s_zt, 2.) * pmtt;

  // Рассчитываем значения из измерения неопределенности
  PLi(0, 0) += pow(c_zt, 2.) * dr + pow(zi_r * s_zt, 2.) * dphi;
  PLi(0, 1) += s_zt * c_zt * dr  - zi_r * zi_r * s_zt * c_zt * dphi;
  PLi(1, 0) = PLi(0, 1);
  PLi(1, 1) += s_zt * s_zt * dr + pow(zi_r * c_zt, 2.) * dphi;

  // Обновляем значение постериорной (прошлой) ковариации
  P.block(ROBOT_STATE_SIZE + landmarkIndex * 2 - 2, ROBOT_STATE_SIZE + landmarkIndex * 2 - 2, 2, 2) = PLi;

  // Выводим информацию о добавленном маяке в состояние
   ROS_INFO("Adding landmark to state, x: %f y: %f", newLandmark[0], newLandmark[1]);

  return landmarks_found_quantity;
}


void Slam::correct(int landmarkIndex, int measurementIndex)
{
  // TODO 
  // Здесь должен быть код для обновления состояния по измерению iого маяка

  // Извлекаем текущие значения состояния и измерения
  double x = X(0);
  double y = X(1);
  double theta = X(2);
  double mi_x = X(ROBOT_STATE_SIZE + landmarkIndex * 2);
  double mi_y = X(ROBOT_STATE_SIZE + landmarkIndex * 2 + 1);
  double zi_r = new_landmarks_measurement[measurementIndex](0);
  double zi_phi = new_landmarks_measurement[measurementIndex](1);

  // Рассчитываем предсказанное измерение
  double dist2 = pow(mi_x - x, 2.0) + pow(mi_y - y, 2.0);
  double dist = sqrt(dist2);
  double x_mi_x = x - mi_x;
  double y_mi_y = y - mi_y;
  double zi_r_pred = dist;
  double zi_phi_pred = angles::normalize_angle(atan2(-y_mi_y, -x_mi_x) - theta);
  Eigen::Vector2d measurementPred(zi_r_pred, zi_phi_pred);

  // Вычисляем якобианы
  Eigen::Matrix<double, 2, 3> Gi_x;
  Gi_x << (x_mi_x / dist), (y_mi_y / dist), 0,
          (-y_mi_y / dist2), (x_mi_x / dist2), -1;

  Eigen::Matrix<double, 2, 2> Gi_m;
  Gi_m << (-x_mi_x / dist), (-y_mi_y / dist),
          (y_mi_y / dist2), (-x_mi_x / dist2);

  Eigen::Matrix<double, 2, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2> Gi;
  Gi.setZero();
  Gi.block(0, 0, 2, 3) = Gi_x;
  Gi.block(0, ROBOT_STATE_SIZE + landmarkIndex * 2, 2, 2) = Gi_m;

  // Вычисляем временную матрицу и корректирующую матрицу Калмана
  Eigen::Matrix<double, 2, 2> tempMat;
  tempMat = Gi * P * Gi.transpose() + Q;

  Eigen::Matrix<double, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2, 2> K;
  K.setZero();
  K = P * Gi.transpose() * tempMat.inverse();

  // Обновляем матрицу ковариации и вектор состояния
  P = (Eigen::Matrix<double, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2, ROBOT_STATE_SIZE + NUMBER_LANDMARKS * 2>::Identity() - K * Gi) * P;
  X = X + K * (new_landmarks_measurement[measurementIndex] - measurementPred);
}

void Slam::on_scan(const sensor_msgs::LaserScan& scan) {
  detect_landmarks(scan);
  predict((scan.header.stamp - last_time).toSec());
  last_time = scan.header.stamp;
  for (std::size_t i = 0; i < new_landmarks.size(); ++i) {
    const auto landmark_index = associate_measurement(new_landmarks[i]);
    if (landmark_index >= 0) {
      correct(landmark_index, i);
    } else {
        if (landmarks_found_quantity < NUMBER_LANDMARKS) {
          add_landmark_to_state(i);
        } else { 
            ROS_ERROR_STREAM("can not associate new landmark with any existing one");
        }
    }
  }
  publish_results("map", scan.header.stamp);
  publish_transform(scan.header);
}


void fill_pose_msg(geometry_msgs::PoseWithCovariance& pose,
                   double x, double y, double fi,
                   const Eigen::Matrix2d& cov_matr) {
  pose.covariance.assign(0);
  pose.covariance[0] = cov_matr(0,0); pose.covariance[1] = cov_matr(0,1);
  pose.covariance[6] = cov_matr(1,0); pose.covariance[7] = cov_matr(1,1);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.w = cos(fi/2);
  pose.pose.orientation.z = sin(fi/2);
}

void fill_pose_msg(geometry_msgs::Pose& pose,
                   double x, double y, double fi) {
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.w = cos(fi/2);
    pose.orientation.z = sin(fi/2);
}


void Slam::publish_results(const std::string& frame, const ros::Time& time) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = time;
  // публикуем сообщение с позицией робота
  fill_pose_msg(pose.pose, X(0), X(1), X(2));
  pose_pub.publish(pose);

  // публикуем сообщения с положениями маяков
  for (int i = 0; i < landmarks_found_quantity; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = time;
    fill_pose_msg(pose.pose, X(ROBOT_STATE_SIZE + i * 2), X(ROBOT_STATE_SIZE + i * 2 + 1), 0);
    landmark_pub[i].publish(pose);
  }
}

void Slam::publish_transform(const std_msgs::Header& scan_header)
{  
  // публикуем трансформ от скана до карты, 
  // не наоборот, так как дерево tf - однонаправленное
  tf::Transform tf_transform;

  double angle = X(2);
  Eigen::Matrix2d R;
  R (0, 0) = R (1, 1) = cos (angle);
  R (0, 1) = -sin (angle);
  R (1, 0) = sin (angle);
  Eigen::Vector2d t = X.head(2);
  Eigen::Isometry2d transform = Eigen::Translation2d(t) * Eigen::Isometry2d(R);

  Eigen::Isometry2d inverted_transform = transform.inverse();

  tf_transform.setOrigin( tf::Vector3(inverted_transform.translation().x(), 
                      inverted_transform.translation().y(), 
                      0.0) );
  tf::Quaternion inv_q;
  const auto& inv_matrix = inverted_transform.matrix();
  double inv_yaw = atan2(inv_matrix(1, 0), inv_matrix(0, 0));
  inv_q.setRPY(0, 0, inv_yaw);
  tf_transform.setRotation(inv_q);

  br.sendTransform(tf::StampedTransform(tf_transform, 
                                        scan_header.stamp, 
                                        scan_header.frame_id, 
                                        map_frame));
}

void Slam::predict(double dt)
{

  X(0) += v * cos(X(2)) * dt;
  X(1) += v * sin(X(2)) * dt;
  X(2) += w * dt;
  X(2) = angles::normalize_angle(X(2));

  // вычисляем якобиан
  A = Eigen::Matrix3d::Identity();
  A(0,0) = 1.0; A(0,1) = 0; A(0,2) = -v * sin(X(2)) * dt;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = v * cos(X(2)) * dt;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1.0;

  // P = A*P*AT + R для блока соответствующего роботу
  P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) =
      A * P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) * A.transpose() + R;
  // для остальных блоков
  P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2) = 
    A * P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2);
  P.bottomLeftCorner(NUMBER_LANDMARKS * 2, ROBOT_STATE_SIZE) = 
    P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2).transpose();
}

void Slam::advertize_landmark_publishers()
{
  std::string landmark("landmark");
  for (int i = 0; i < NUMBER_LANDMARKS; ++i)
  {
    std::stringstream stream;
    // landmark0 landmark1 ...
    stream << landmark << i;
    landmark_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(stream.str(), 1);
  }
}

Slam::Slam():
    nh("~"),
    odo_sub(nh.subscribe("/odom", 1, &Slam::on_odo, this)),
    scan_sub(nh.subscribe("/scan", 1, &Slam::on_scan, this)),
    pose_pub(nh.advertise<geometry_msgs::PoseStamped>("slam_pose", 1)),
    X(ROBOT_STATE_SIZE + 2*NUMBER_LANDMARKS),
    A(Eigen::Matrix3d::Identity()),
    P(Eigen::MatrixXd::Zero(X.size(), X.size()))
{
  // начальный вектор состояния заполняем нулями
  X = Eigen::VectorXd::Zero(X.size());
  // записываем огромное значение начальной ковариации для маяков
  P.bottomRightCorner(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2) =
      HUGE_COVARIANCE * Eigen::MatrixXd::Identity(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2);

  advertize_landmark_publishers();

  Q = Eigen::Matrix2d::Zero();
  Q(0, 0) = nh.param<double>("range_sigma_sqr", 0.01);
  Q(1, 1) = nh.param<double>("angle_sigma_sqr", 0.001);
  R = Eigen::Matrix3d::Zero();
  R(0, 0) = nh.param<double>("x_sigma_sqr", 0.001);
  R(1, 1) = nh.param<double>("y_sigma_sqr", 0.001);
  R(2, 2) = nh.param<double>("angle_sigma_sqr", 0.0001);

  std::cout.precision(4);
}
