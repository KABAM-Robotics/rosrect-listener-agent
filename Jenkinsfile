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
            sh '''
            . /opt/ros/melodic/setup.sh
          #  cd catkin_ws_rosrect_listener/
          #  catkin_make run_tests_rosrect-listener-agent
            '''
      }
    }

  }
}
