"use strict";

import React from 'react';
import { withRouter, Link } from 'react-router-dom';
import {Navbar, Nav, NavItem } from 'react-bootstrap';
import UserService from '../services/UserService';
import {AccessibleFakeButton, Avatar, DropdownMenu, FontIcon, IconSeparator, ListItem, Divider} from "react-md";
import logo from "../assets/img/logo.svg";

class Header extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            user: UserService.isAuthenticated() ? UserService.getCurrentUser() : undefined
        }
    }

    logout() {
        UserService.logout();
        this.state = {
            user: UserService.isAuthenticated() ? UserService.getCurrentUser() : undefined
        };
        window.location.reload();
    }

    render() {
        const PlatformName = <Navbar.Header><Navbar.Brand className="wayerName"><Link to={'/'}><img src={logo} className="headerLogo"/></Link></Navbar.Brand></Navbar.Header>;
        if(this.state.user){
            const loggedInUser = <AccessibleFakeButton component={IconSeparator}
                                                       iconBefore
                                                       label={<IconSeparator label={this.state.user.username} className="boldPText"><FontIcon>arrow_drop_down</FontIcon></IconSeparator>}>{/*<Avatar random>{this.state.user.username.substring(0,1).toUpperCase()}</Avatar>*/}</AccessibleFakeButton>;
            if(this.state.user.role === "company"){
                return (
                    <Navbar>
                        <Navbar.Header><Navbar.Brand className="wayerName"><Link to={'/company'}><img src={logo} className="headerLogo"/></Link></Navbar.Brand></Navbar.Header>
                        <Nav pullRight>
                            <NavItem eventKey={1} onClick={() => this.props.history.push('/company/search')}><p className="boldText">Find Talents</p></NavItem>
                            <NavItem eventKey={2} style={{paddingLeft:30+'px'}} onClick={() => this.props.history.push('/company/projects')}><p className="boldText">My Projects</p></NavItem>
                            <NavItem eventKey={3} style={{paddingLeft:30+'px'}} onClick={() => this.props.history.push('/aboutUs')}><p className="boldText">About Us</p></NavItem>
                            <NavItem eventKey={4} style={{paddingLeft:50+'px'}}>
                                <DropdownMenu
                                    id={`avatar-dropdown-menu`}
                                    menuItems={[<ListItem primaryText="Add Project" className="boldPText" onClick={() => this.props.history.push('/company/project/create')}/>, { divider: true }, <ListItem primaryText="Logout" className="boldPText" onClick={() => this.logout()}/>]}
                                    anchor={{
                                        x: DropdownMenu.HorizontalAnchors.CENTER,
                                        y: DropdownMenu.VerticalAnchors.OVERLAP}}
                                    position={DropdownMenu.Positions.BOTTOM_LEFT}
                                    animationPosition="below"
                                    sameWidth
                                >
                                    {loggedInUser}
                                </DropdownMenu>
                            </NavItem>
                        </Nav>
                    </Navbar>
                );

            }else{
                return (
                    <Navbar>
                        {PlatformName}
                        <Nav pullRight>
                            <NavItem eventKey={1} onClick={() => this.props.history.push('/project/search')}><p className="boldText">Find Projects</p></NavItem>
                            <NavItem eventKey={2} style={{paddingLeft:50+'px'}}onClick={() => this.props.history.push('/aboutUs')}><p className="boldText">About Us</p></NavItem>
                            <NavItem eventKey={3} style={{paddingLeft:70+'px'}}>
                                <DropdownMenu
                                    id={`avatar-dropdown-menu`}
                                    menuItems={[<ListItem primaryText="Profile" className="boldPText" onClick={() => this.props.history.push('/profile')}/>, { divider: true }, <ListItem primaryText="Logout" className="boldPText" onClick={() => this.logout()}/>]}
                                    anchor={{
                                        x: DropdownMenu.HorizontalAnchors.CENTER,
                                        y: DropdownMenu.VerticalAnchors.OVERLAP}}
                                    position={DropdownMenu.Positions.BOTTOM_LEFT}
                                    animationPosition="below"
                                    sameWidth
                                >
                                    {loggedInUser}
                                </DropdownMenu>
                            </NavItem>
                        </Nav>
                    </Navbar>
                );

            }
        }else{
            return (
                <Navbar>
                    {/*<div className="col-md-12 header">
                    <div className="col-md-2 text-center"><Link to={'/'}><img className="headerLogo" src={logo}/></Link></div>
                    <div className="col-md-4"/>
                    <div className="col-md-2"><Link to={'/project/search'}><p className="headerLink boldText">Find Projects</p></Link></div>
                    <div className="col-md-2"><Link to={'/company/search'}><p className="headerLink boldText">For Companies</p></Link></div>
                    <div className="col-md-1"><p className="headerLink boldText">About</p></div>
                    <div className="col-md-1"><Link to={'/login'}><p className="headerLink boldText">Login</p></Link></div>
                    </div>*/}
                    {PlatformName}
                    <Nav pullRight>
                        <NavItem eventKey={1} onClick={() => this.props.history.push('/company')}><p className="boldText">For Companies</p></NavItem>
                        <NavItem eventKey={2} style={{paddingLeft:50+'px'}}onClick={() => this.props.history.push('/aboutUs')}><p className="boldText">About Us</p></NavItem>
                        <NavItem eventKey={3} style={{paddingLeft:100+'px'}} onClick={() => this.props.history.push('/login')}><p className="boldText">Sign Up</p></NavItem>
                    </Nav>
                </Navbar>
            );
        }
    }
}

export default withRouter(Header);
