# [Intel]AI+SW 아카데미 최종 종합 프로젝트

## Source

### 이 폴더에는 프로젝트에 사용되는 소스코드들이 업로드됩니다.

### 업로드 방법

#### 1. Window에서는 GIT BASH에서, Linux에서는 터미널에서 깃 클론을 먼저 해주세요.

`$ git clone https://github.com/KINGMINWOO/INTEL_AI_SW_LAST_PROJECT.git`

#### 2. 폴더로 이동하고 깃 상태를 확인합니다. -> 브랜치는 현재 `main`으로 되어있습니다.
```
$ cd INTEL_AI_SW_LAST_PROJECT
$ git status // On branch main ...
```
#### 2-1. 만약 깃 정보가 입력되이 있지 않으면 등록해주세요.
```
// System(내PC기준)에서 사용할 전역 User Name 등록
// git config --global user.name <name>
$ git config --global user.name "KINGMINWOO"

// System(내PC기준)에서 사용할 전역 User Email 등록
// git config --global user.email <email>
$ git config --global user.email "zhongbi04@gmail.com"
```

#### 3. 본인 이름의 이니셜로 브렌치를 생성하고 생성된 브랜치로 이동해주세요.
```
$ git branch kmw(본인 이니셜)
$ git checkout kmw // 해당 브렌치로 이동
$ git status // On branch kmw
```
### (필수!!!!) 프로젝트 진행 중 코드 업로드는 본인 브랜치에서 업로드해주세요.

#### 4. 생성된 폴더에 소스코드 넣고 본인 브랜치에 푸시(push)해주세요.
```
$ git add .
$ git commit -m "변경사항"
$ git push origin knw(본인 브랜치)
```
#### 4-1. 푸시 과정에서 충돌이 발생하면 팀장을 불러주세요.

#### 5. 깃허브로 와서 PR(Pull requests)해주세요.

### Reviewer를 팀장(KINGMINWOO)으로 설정해주세요.

#### 6. 추가로 소스코드를 업로드하고 싶으실 때 메인 브랜치 풀(pull) 한번 하고 업로드 해주세요.
```
$ git pull origin main
```

