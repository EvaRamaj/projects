"use strict";

import React from 'react';
import { BrowserRouter as Router, Route, Switch, Redirect, browserHistory } from 'react-router-dom';

import { HomePageView } from './views/HomePageView';
import { ImpressumView } from './views/ImpressumView';
import { PrivacyView } from './views/PrivacyView';
import { LoginView } from './views/LoginView';
import HomePageCompany from './components/HomePageCompany';
import { ProjectSearchView } from './views/ProjectSearchView';
import { CompanySearchView } from './views/CompanySearchView';
import { CompanySignUpView } from './views/CompanySignUpView';
import { CompanyOnboardingView } from './views/CompanyOnboardingView';
import { MovieFormView }   from './views/MovieFormView';
import { UserSignupView } from "./views/UserSignupView";
import { UserProfileView } from "./views/UserProfileView";
import {UserDetailView} from "./views/UserDetailView";
import {UserContactView} from "./views/UserContactView";
import {ConfirmEmailView} from "./views/ConfirmEmailView";
import { LinkedInPopUp } from 'react-linkedin-login-oauth2';
import {ProjectDetailView} from "./views/ProjectDetailView";
import {ProjectsListView} from "./views/ProjectsListView";
import {NewProjectView} from "./views/NewProjectView";
import {MatchingResultsView} from "./views/MatchingResultsView";
import {AboutUsView} from "./views/AboutUsView";


import UserService from "./services/UserService";


export default class App extends React.Component
{
    constructor(props) {
        super(props);

        this.state = {
            title: 'Xperienced',
            routes: [
                { component: HomePageView , path: '/', exact: true},
                { component: LinkedInPopUp , path: '/auth/linkedin/callback', exact: true},
                { component: ImpressumView , path: '/impressum', exact: true},
                { component: AboutUsView , path: '/aboutUs', exact: true},
                { component: PrivacyView , path: '/privacy', exact: true},
                { component: HomePageCompany , path: '/company', exact: true},
                { render: (props) => {
                        if(!UserService.isAuthenticated()) {
                            return (<LoginView {...props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }} , path: '/login', exact: true},
                { component: ConfirmEmailView , path: '/confirmEmail'},
                { component: ProjectSearchView , path: '/project/search'},
                { component: CompanySearchView , path: '/company/search'},
                { component: ProjectDetailView , path: '/project/:id'},
                { component: CompanyOnboardingView , path: '/company/onboarding'},
                { component: ProjectsListView , path: '/company/projects'},
                { component: MatchingResultsView , path: '/company/recommendations/:id'},
                { render: (props) => {
                        if(!UserService.isAuthenticated()) {
                            return (<CompanySignUpView {...props} />)
                        }
                        else {
                            return (<Redirect to={'/company/project/create'}/>)
                        }} , path: '/company/login', exact: true},
                { render: (props) => {
                        if(UserService.isAuthenticated()) {
                            if (UserService.getCurrentUser().role==="company") {
                                return (<UserContactView {...props} />)
                            }
                            else {
                                return (<Redirect to={'/'}/>)
                            }
                        }
                        else {
                            return (<Redirect to={'/company/login'}/>)
                        }} , path: '/company/user/contact', exact: true},
                { render: (props) => {
                        if(UserService.isAuthenticated()) {
                            if (UserService.getCurrentUser().role==="company") {
                                return (<UserDetailView {...props} />)
                            }
                            else {
                                return (<Redirect to={'/'}/>)
                            }
                        }
                        else {
                            return (<Redirect to={'/company/login'}/>)
                        }} , path: '/company/user/:id'},
                { render: (props) => {
                        if(UserService.isAuthenticated()) {
                            if (UserService.getCurrentUser().role==="company") {
                                return (<NewProjectView {...props} />)
                            }
                            else {
                                return (<Redirect to={'/'}/>)
                            }
                        }
                        else {
                            return (<Redirect to={'/company/login'}/>)
                        }} , path: '/company/project/create'},
                { render: (props) => {
                        if(UserService.isAuthenticated()) {
                            return (<MovieFormView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/login'}/>)
                        }} , path: '/edit/:id'},
                { render: (props) => {
                        if(!UserService.isAuthenticated()) {
                            return (<UserSignupView {...props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }} , path: '/register', exact: true},
                { render: (props) => {
                        if(UserService.isAuthenticated()) {
                                return (<UserProfileView {...props} />)
                        }
                        else {
                            return (<Redirect to={'/login'}/>)
                        }} , path: '/profile', exact: true},
            ]
        };
    }

    componentDidMount(){
        document.title = this.state.title;
    }

    render() {
        return(
            <div>
                <Router history={browserHistory}>
                    <Switch>
                        {this.state.routes.map((route, i) => (<Route key={i} {...route}/>) )}
                    </Switch>
                </Router>
            </div>
        );
    }
}
