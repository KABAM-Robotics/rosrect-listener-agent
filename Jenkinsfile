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
            ls
            sh test.sh
            '''

      }
    }
    stage ("--Extract test results--") {
    steps {
    cobertura coberturaReportFile: 'myxmlfile.xml'
    }
    }
  }

}
