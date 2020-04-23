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
            cd rosrect-listener-agent
            sh 'catkin_make run_tests_rosrect-listener-agent'
      }
    }

  }
}
