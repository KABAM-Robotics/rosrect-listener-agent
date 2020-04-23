pipeline{
  agent { label 'ubuntu' }
  stages{
    stage('--test--'){
      steps{
        echo 'conducting tests'
      }
    }
    stage('--build--'){
      steps{
            sh 'cd catkin_ws_rosrect_listener && pwd && source /opt/ros/melodic/setup.bash && catkin_make run_tests_rosrect-listener-agent'
      }
    }

  }
}
