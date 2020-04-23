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
            sh ''' /bin/bash -l -c 'pre-build'
            . ~./bashrc
            '''

      }
    }

  }
}
