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
            sh '. /opt/ros/melodic/setup.sh'

      }
    }

  }
}
