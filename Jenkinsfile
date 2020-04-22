pipeline{
  agent any
  stages{
    stage('--test--'){
      steps{
        echo 'conducting tests'
      }
    }
    stage('--build--'){
      steps{
        sh 'catkin_make run_tests_rosrect-listener-agent'
      }
    }

  }
}
