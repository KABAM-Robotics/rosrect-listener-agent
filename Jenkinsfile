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
        sh 'git clone https://github.com/cognicept-admin/rosrect-listener-agent.git'
      }
    }

  }
}
