"use strict";
import Navigation from './components/Navigation';
import React from 'react';
import { HashRouter as Router, Route, Switch, Redirect } from 'react-router-dom';
import { library } from '@fortawesome/fontawesome-svg-core'

// -----------------------------------------------------------------
// icons
import { faUsers } from '@fortawesome/free-solid-svg-icons';
import { faUserCog } from '@fortawesome/free-solid-svg-icons';
import { faTrashAlt  } from '@fortawesome/free-solid-svg-icons';
import { faCheck  } from '@fortawesome/free-solid-svg-icons';
import { faTimes  } from '@fortawesome/free-solid-svg-icons';
import { faClipboardList } from '@fortawesome/free-solid-svg-icons';
import { faFileUpload } from '@fortawesome/free-solid-svg-icons';
import { faEnvelope } from '@fortawesome/free-solid-svg-icons';
import { faCog } from '@fortawesome/free-solid-svg-icons';
import { faStar } from '@fortawesome/free-solid-svg-icons';
import { faPlusSquare } from '@fortawesome/free-solid-svg-icons';
import { faShoppingCart } from '@fortawesome/free-solid-svg-icons';
import { faSyncAlt  } from '@fortawesome/free-solid-svg-icons';

library.add(faUsers);
library.add(faUserCog);
library.add(faTrashAlt);
library.add(faCheck);
library.add(faTimes);
library.add(faClipboardList);
library.add(faFileUpload);
library.add(faEnvelope);
library.add(faCog);
library.add(faStar);
library.add(faPlusSquare);
library.add(faShoppingCart );
library.add(faSyncAlt );
// -----------------------------------------------------------------


import { MovieListView } from './views/MovieListView';
import { MovieFormView } from './views/MovieFormView';
import { ItemListView } from './views/ItemListView';
import { SearchView } from './views/SearchView';
import { BookingListView } from './views/BookingListView';
import { ConversationListView } from './views/ConversationListView';
import { SearchItemDetailView } from './views/SearchItemDetailView';
import { MyItemDetailView } from './views/MyItemDetailView';
import { ItemFormView } from './views/ItemFormView';
import { BecomeLessorFormView } from './views/BecomeLessorFormView';
import { UserLoginView } from "./views/UserLoginView";
import { UserSignupView } from "./views/UserSignupView";
import { UserListView } from "./views/UserListView";
import { UserDetailView } from "./views/UserDetailView";
import { UserFormView } from './views/UserFormView';
// import { GrantLessorPrivilegesView } from './views/GrantLessorPrivilegesView';
import { HomepageView } from "./views/HomepageView";
import { AdminDashboardView } from "./views/AdminDashboardView";
import { UserDashboardView } from "./views/UserDashboardView";
import { CategoryListView } from "./views/CategoryListView";
import { CategoryFormView } from "./views/CategoryFormView";
import { ConversationFormView } from "./views/ConversationFormView";
import { ConversationFormUserView } from "./views/ConversationFormUserView";
import { BookingDetailView } from "./views/BookingDetailView";
import { ConversationDetailView } from "./views/ConversationDetailView";
import { UserListLessorView } from "./views/UserListLessorView";
import { GrantLessorPrivilegesView } from "./views/GrantLessorPrivilegesView";

import { LogoutView } from "./views/LogoutView";
import { Footer } from "./components/Footer";

// import {UserEvaluationFormView} from "./views/UserEvaluationFormView";
import {UserEvaluationDetailView} from "./views/UserEvaluationDetailView";
import {UserEvaluationListView} from "./views/UserEvaluationListView";
// import {ItemEvaluationListView} from "./views/ItemEvaluationListView";


import Example from "./components/Notifications";

import AuthService from "./services/AuthService";
import { NavigationView } from "./views/NavigationView";



export default class App extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            name: 'LeazIt App',
            routes: [
                { component: HomepageView , path: '/', exact: true},
                { component: MovieListView , path: '/movies', exact: true},
                { component: ItemListView , path: '/items', exact: true},
                { component: SearchItemDetailView , path: '/item/:id'},
                { component: UserListView , path: '/users'},
                { component: UserDetailView , path: '/profile/:id'},
                { component: UserDetailView , path: '/my_profile'},
                { component: UserEvaluationListView, path: '/user_evaluations'},
                { component: UserEvaluationDetailView, path: '/user_evaluation/:id'},
                // { component: ItemEvaluationListView, path: '/item_evaluations'},
                { component: CategoryListView , path: '/categories'},
                { component: BookingDetailView , path: '/booking/:id'},
                { component: BookingDetailView , path: '/items/:itemId/booking/:id'},
                { component: Example , path: '/notifications'},

                { render: (props) => {
                        if(AuthService.getRole()==='Lessor') {
                            return (<MyItemDetailView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }} , path: '/my_item/:id'},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<ConversationDetailView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }} , path: '/conversation/:id'},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<ConversationListView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }} , path: '/my_conversations'},
                { render: (props) => {
                    if(AuthService.isAuthenticated()) {
                        return (<UserEvaluationFormView {... props} />)
                    }
                    else {
                        return (<Redirect to={'/'}/>)
                    }} , path: '/create_evaluation'},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<ReplyFormView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/reply/'},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<ConversationFormView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/new_conversation/'},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<ConversationFormUserView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/new_user_conversation/:username'},
                { render: (props) => {
                        if(AuthService.getRole()==='Lessor'|| AuthService.getRole() === 'Lessee') {
                            return (<BookingListView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/my_bookings'},
                { render: (props) => {
                        if(AuthService.getRole()==='Lessor') {
                            return (<BookingListView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/my_item_bookings/:id'},
                // { render: (props) => {
                //         if(AuthService.getRole()==='Admin') {
                //             return (<GrantLessorPrivilegesView {... props} />)
                //         }
                //         else {
                //             return (<Redirect to={'/my_profile'}/>)
                //         }} , path: '/id_doc/:id'},
                { render: (props) => {
                        if(AuthService.getRole()==='Admin') {

                            return (<GrantLessorPrivilegesView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/id_doc/:id'},
                { render: (props) => {
                        if(AuthService.getRole()==='Admin') {
                            return (<UserListLessorView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/admin/view_lessor_requests'},
                { render: (props) => {
                        if(AuthService.getRole()==='Lessor') {
                            return (<ItemListView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/my_items'},
                { render: (props) => {
                        if(AuthService.getRole()==='Lessee') {
                            return (<BecomeLessorFormView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/my_profile'}/>)
                        }} , path: '/become_lessor'},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<UserFormView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/login'}/>)
                        }} , path: '/edit_profile'},
                { render: (props) => {
                        if(AuthService.getRole() === 'Admin') {
                            return (<AdminDashboardView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }} , path: '/admin_dashboard'},
                { render: (props) => {
                        console.log(AuthService.getRole());
                        if(AuthService.getRole() === 'Lessor' || AuthService.getRole() === 'Lessee') {
                            return (<UserDashboardView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }} , path: '/user_dashboard'},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<MovieFormView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/login'}/>)
                        }} , path: '/edit/:id'},
                { render: (props) => {
                    if(AuthService.isAuthenticated()) {
                        return (<ItemFormView {... props} />)
                    }
                    else {
                        return (<Redirect to={'/login'}/>)
                    }}, path: '/add',},
                { render: (props) => {
                    return (<SearchView {... props} />)
                    }, path: '/results/:str',},
                { render: (props) => {
                        if(AuthService.getRole() === 'Admin') {
                            return (<CategoryFormView {... props} />)
                        }
                        else {
                            return (<Redirect to={'/'}/>)
                        }}, path: '/add_category',},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            return (<Redirect to={'/user_dashboard'}/>)
                        }
                        else {
                            return (<UserLoginView {... props} />)
                        }}, path: '/login',},
                { render: (props) => {
                        if(AuthService.isAuthenticated()) {
                            console.log('auth: ', AuthService.getRole());

                            return (<Redirect to={'/logout'}/>)
                        }
                        else {
                            return (<UserSignupView {... props} />)
                        }}, path: '/register',},



                // { component: UserSignupView, path: '/register'},
                { component: LogoutView, path: '/logout'}
            ]
        };
    }

    componentDidMount(){
        document.title = this.state.name;
    }

    render() {
        return(
            <div>
                <NavigationView/>
                <div className="container">
                    <Router>
                        <Switch>
                            {this.state.routes.map((route, i) => (<Route key={i} {...route}/>) )}
                        </Switch>
                    </Router>
                </div>
                <Footer/>
            </div>
        );
    }
}


