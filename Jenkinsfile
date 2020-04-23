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
            sh 'cd catkin_ws_rosrect_listener'
            sh 'catkin_make run_tests_rosrect-listener-agent'
      }
    }

  }
}
